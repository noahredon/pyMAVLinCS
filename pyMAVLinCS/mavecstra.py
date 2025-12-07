# pyMAVLinCS/mavecstra.py
# Copyright (C) 2025 Noah Redon
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import os
import threading
import time
import struct
import logging
from math import nan
from collections import deque
from pymavlink import mavutil


class MavThread:
    """Manages Threads."""
    def __init__(self,
            master: mavutil.mavfile,
            logger: logging.Logger,
            queue_maxsize: int = 20,
            msg_timeout: float = 0.1
        ):
        """Manages Threads.

        Args:
            master (mavutil.mavfile): Master connection.
            logger (logging.Logger): Logger to use.
            queue_maxsize (int): Maximum size of the message queue.
            msg_timeout (float): Timeout in seconds to consider a received message as too old.
        """
        logger.debug("Initializing MavThread..")

        self._logger: logging.Logger = logger
        self._master: mavutil.mavfile = master

        # Lock for sending messages in multi-threaded context
        self._send_lock = threading.Lock()

        # Thread
        self._thread: threading.Thread | None = None

        # Arguments for proper thread shutdown
        self._disable_thread: threading.Event = threading.Event()
        self._thread_disabled: threading.Event = threading.Event()

        # Thread is initially disabled
        self._thread_disabled.set()

        self._lock: threading.Lock = threading.Lock() # To prevent certain conflicts

        self._queue_maxsize: int = max(1, int(queue_maxsize))
        self._queue: deque = deque(maxlen=self._queue_maxsize)

        self._msg_received: threading.Event = threading.Event()
        self._last_msg_timeout: float = msg_timeout

        logger.debug("MavThread initialized")

    def __send(self, mavmsg, force_mavlink1: bool = False) -> None:
        """send a MAVLink message, multi-thread safe"""
        if self._master is None:
            return
        with self._send_lock:
            buf = mavmsg.pack(self._master.mav, force_mavlink1=force_mavlink1)
            self._master.write(buf)
            self._master.mav.seq = (self._master.mav.seq + 1) % 256
            self._master.mav.total_packets_sent += 1
            self._master.mav.total_bytes_sent += len(buf)
        if self._master.mav.send_callback is not None and self._master.mav.send_callback_args is not None and self._master.mav.send_callback_kwargs is not None:
            self._master.mav.send_callback(mavmsg, *self._master.mav.send_callback_args, **self._master.mav.send_callback_kwargs)

    def set_master(self, master: mavutil.mavfile):
        """Sets the master to use.

        Args:
            master (mavutil.mavfile): Master connection.
        """
        self._master = master
        # Adding a threading.Lock to the master.mav.send method
        self._master.mav.send = self.__send

    def is_thread_active(self) -> bool:
        """Indicates whether the thread is active.

        Returns:
            bool: True if the thread is active, False otherwise.
        """
        return not self._thread_disabled.is_set()

    def start_thread(self):
        """Starts the Thread."""
        if not self.is_thread_active(): # Thread is closed
            self._logger.debug("Starting Thread..")
            self._thread_disabled.clear()
            self._disable_thread.clear()
            self._thread = threading.Thread(target=self._thread_func, args=(), daemon=True)
            self._thread.start() # Begin listening to messages + sending HEARTBEATs
            self._logger.debug("Thread started")
        else:
            self._logger.warning("Thread already started but attempting to start it: aborting")

    def stop_thread(self):
        """Stops the Thread."""
        if self.is_thread_active(): # Thread is open
            self._logger.debug("Closing Thread..")
            self._disable_thread.set()
            self._thread_disabled.wait()
            if self._thread is not None:
                self._thread.join(timeout=0.3)
            self._logger.debug("Thread closed")
        else:
            self._logger.warning("Thread already closed but attempting to close it: aborting")

    def recv_msg(self):
        '''Message receive routine (same logic as the pymavlink one)
        
        Traverses the queue from oldest to newest:
        - if a message is too old (t - timestamp > last_msg_timeout) -> remove it and continue
        - if a valid message is found -> return it (newer messages remain in the queue)
        - if no valid message -> queue becomes empty and return None
        '''
        t = time.time()

        with self._lock:
            # Traverse from the head (oldest)
            while self._queue:
                m = self._queue[0]  # oldest
                # if too old -> remove and continue
                if t - getattr(m, "_timestamp", 0) > self._last_msg_timeout:
                    self._queue.popleft()
                    continue
                # otherwise message is valid: remove and return it
                valid_msg = self._queue.popleft()
                # if queue is now empty, clear the event
                if not self._queue:
                    self._msg_received.clear()
                return valid_msg

            # if we reach here, no valid messages left (queue empty or all too old)
            self._msg_received.clear()
            return None

    def select(self, timeout):
        '''Wait for up to timeout seconds for more data (same logic as the pymavlink one)'''
        self._msg_received.wait(timeout=timeout)

    def _thread_func(self):
        try:
            if self._master.source_component in [mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER, mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER2, mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER3, mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER4]:
                mav_type = mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER
            elif self._master.source_component == mavutil.mavlink.MAV_COMP_ID_GPS:
                mav_type = mavutil.mavlink.MAV_TYPE_GPS
            else:
                mav_type = mavutil.mavlink.MAV_TYPE_GCS
            t_last_hb = 0

            while not self._disable_thread.is_set(): # As long as we want to stay connected
                try:
                    t = time.time()
                    if t - t_last_hb >= 1:
                        self._master.mav.heartbeat_send(mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        t_last_hb = t
                except Exception:
                    if not self._disable_thread.is_set():
                        self._logger.exception('Error while sending HEARTBEAT')

                if self._disable_thread.is_set():
                    break

                try:
                    m = self._master.recv_msg()
                    if m:
                        with self._lock:
                            # Add to the bounded queue. If full, deque(maxlen) will automatically remove the oldest.
                            self._queue.append(m)
                            # Signal that a message is available
                            self._msg_received.set()
                            self._msg_received.clear()
                except Exception:
                    if not self._disable_thread.is_set():
                        self._logger.exception('Error while receiving MAVLink message')

                if self._disable_thread.is_set():
                    break

                try:
                    self._master.select(0.05)
                except Exception:
                    time.sleep(0.05)
        except Exception:
            if not self._disable_thread.is_set():
                self._logger.exception('Error in Thread')
        finally:
            self._thread_disabled.set()


class _PendingText(object):
    """Handles the reception of a single statustext"""
    def __init__(self):
        self.expected_count = None
        self.severity = None
        self.chunks = {}
        self.start_time = time.time()
        self.last_chunk_time = time.time()

    def add_chunk(self, m): # m is a statustext message
        self.severity = m.severity
        self.last_chunk_time = time.time()
        if hasattr(m, 'chunk_seq'):
            # mavlink extensions are present.
            chunk_seq = m.chunk_seq
            mid = m.id
        else:
            # Note that m.id may still exist!  It will
            # contain the value 253, STATUSTEXT's mavlink
            # message id.  Thus our reliance on the
            # presence of chunk_seq.
            chunk_seq = 0
            mid = 0
        self.chunks[chunk_seq] = m.text

        if len(m.text) != 50 or mid == 0:
            self.expected_count = chunk_seq + 1

    def complete(self):
        return (self.expected_count is not None and
                self.expected_count == len(self.chunks))

    def accumulated_statustext(self):
        next_expected_chunk = 0
        out = ""
        for chunk_seq in sorted(self.chunks.keys()):
            if chunk_seq != next_expected_chunk:
                out += " ... "
                next_expected_chunk = chunk_seq
            if isinstance(self.chunks[chunk_seq], str):
                out += self.chunks[chunk_seq]
            else:
                out += self.chunks[chunk_seq].decode(errors="ignore")
            next_expected_chunk += 1

        return out


class StatustextReceiver:
    """Manages the reception of statustext messages split into chunks"""
    def __init__(self):
        """Initializes the object"""
        self._statustexts_by_sysidcompid: dict[tuple[int, int], dict[int, _PendingText]] = {}
        self._last_statustext: dict = {}
    
    def last_statustext_from_sysidcompid(self, key: tuple[int, int]):
        """Returns the last complete statustext received from (sysid, compid).

        Args:
            key (tuple[int, int]): (sysid, compid)

        Returns:
            MAVLink_message | None: Last complete STATUSTEXT received. None if no message exists.
        """
        return self._last_statustext.get(key, None)
    
    def parse(self, m):
        """Adds a STATUSTEXT message to the receiver

        Args:
            m (MAVLink_message): Latest STATUSTEXT received.

        Returns:
            MAVLink_message | None: Complete STATUSTEXT or None if incomplete.
        """
        key = (m.get_srcSystem(), m.get_srcComponent())
        if key not in self._statustexts_by_sysidcompid:
            self._statustexts_by_sysidcompid[key] = {}
        if hasattr(m, 'chunk_seq'):
            mid = m.id
        else:
            # m.id will have the value of 253, STATUSTEXT mavlink id
            mid = 0
        if mid not in self._statustexts_by_sysidcompid[key]:
            self._statustexts_by_sysidcompid[key][mid] = _PendingText()

        pending: _PendingText = self._statustexts_by_sysidcompid[key][mid]
        pending.add_chunk(m)
        if pending.complete():
            # all chunks received!
            out = pending.accumulated_statustext()
            if key not in self._last_statustext or out != self._last_statustext[key].text or time.time() > self._last_statustext[key]._timestamp + 2:
                m.text = out
                self._last_statustext[key] = m
                return m
            del self._statustexts_by_sysidcompid[key][mid]
        return None


class TimesyncHandler:
    """Manages the TIMESYNC protocol"""
    def __init__(self,
            master: mavutil.mavfile,
            send_automatic_timesync: bool,
            timesync_delay: int,
            logger: logging.Logger
        ):
        """Initializes the object.

        Args:
            master (mavutil.mavfile): Master connection.
            send_automatic_timesync (bool): Whether to send TIMESYNC automatically.
            timesync_delay (int): Waiting time between two TIMESYNC messages.
            logger (logging.Logger): Logger to use.
        """
        self._logger: logging.Logger = logger
        self._master: mavutil.mavfile = master
        self._latence_ms_by_sysidcompid: dict[tuple[int, int], float] = {}
        self._all_ts1_sent: list[float] = []
        self._send_automatic_timesync: bool = send_automatic_timesync
        self._timesync_delay: int = timesync_delay
        self._nb_heartbeat_since_last_timesync: int = timesync_delay # Counts HEARTBEAT messages for TIMESYNC
    
    @property
    def _ts1(self) -> int:
        return time.time_ns()

    def set_master(self, master: mavutil.mavfile):
        """Sets the master to use.

        Args:
            master (mavutil.mavfile): Master connection.
        """
        self._master = master

    def send_timesync(self) -> bool:
        """Sends a TIMESYNC broadcast to obtain connection latencies.

        Returns:
            bool: True if the command was sent, False otherwise.
        """
        # To check different latencies, use latency = self.get_latence_ms_with_sysidcompid(key)
        if self._master is None:
            self._logger.error("Master object is not defined!")
            return False
        self._master.mav.timesync_send(0, self._ts1)
        return True
    
    def timesync_sent_callback(self, m):
        """Call on each TIMESYNC sent.

        Args:
            m (MAVLink_message): TIMESYNC message sent.
        """
        if m.tc1 == 0: # Sending a request
            if m.ts1 not in self._all_ts1_sent:
                self._all_ts1_sent.append(m.ts1)
            if len(self._all_ts1_sent) > 10:
                self._all_ts1_sent.pop(0) # Prevent list from growing too large
    
    def heartbeat_sent_callback(self):
        """Call on each HEARTBEAT sent."""
        if self._send_automatic_timesync:
            if self._nb_heartbeat_since_last_timesync >= self._timesync_delay:
                self.send_timesync()
                self._nb_heartbeat_since_last_timesync = 1
            else:
                self._nb_heartbeat_since_last_timesync += 1
    
    def parse(self, m) -> bool:
        """Call upon reception of a TIMESYNC message.

        Args:
            MAVLink_message: TIMESYNC message received.

        Returns:
            bool: True if it is a response to one of our TIMESYNC messages, False otherwise.
        """
        tc1 = m.tc1
        ts1 = m.ts1
        if ts1 in self._all_ts1_sent and tc1 != 0: # Response to one of our TIMESYNC
            round_trip_ns = max(time.time_ns() - ts1, 0) # Avoid internal clock errors
            latency_ns = round_trip_ns // 2
            key = (m.get_srcSystem(), m.get_srcComponent())
            latency_ms = int(latency_ns // 1e6)
            self._latence_ms_by_sysidcompid[key] = latency_ms
            if not self._send_automatic_timesync:
                self._logger.debug("New latency with %s: %s ms", key, latency_ms)
            return True
        else:
            if m.tc1 == 0: # It is a request
                self._master.mav.timesync_send(self._ts1, m.ts1)
            return False
    
    def get_latence_ms_with_sysidcompid(self, key: tuple[int, int]) -> int:
        """Returns the estimated one-way latency between our system and (sysid, compid) in ms.
        
        Args:
            key (tuple[int, int]): (sysid, compid)
        
        Returns:
            int: Latency in ms. NaN if no data received.
        """
        return self._latence_ms_by_sysidcompid.get(key, nan)


class MavLogger:
    """Handles logging to a file"""
    def __init__(self, file: str | None, relative_path_to_cwd: bool = True):
        """Initializes the object

        Args:
            file (str | None): Name of the tlog file.
            relative_path_to_cwd (bool): Path relative to current working directory or not.
        """
        self.file = file
        if file is not None:
            # Ensure .tlog suffix
            if not file.endswith(".tlog"):
                file += ".tlog"

            if relative_path_to_cwd:
                cwd = os.getcwd()
                tlog_path = os.path.join(cwd, file)
            else:
                tlog_path = file

            self._lock = threading.Lock()
            # Open once in append mode; creates file if it doesn't exist
            self._tlog_handle = open(tlog_path, "wb")  # clear existing
            self._tlog_handle.close()
            self._tlog_handle = open(tlog_path, "ab")  # keep open for append

    def log(self, m):
        """Logs a MAVLink message.

        Args:
            m (MAVLink_message): MAVLink message.
        """
        if self.file is not None and m.get_type() != 'BAD_DATA':
            ts = int(time.time() * 1e6) & ~3  # microseconds aligned to 4
            binary_data = struct.pack(">Q", ts) + m.get_msgbuf()
            with self._lock:
                self._tlog_handle.write(binary_data)
                self._tlog_handle.flush()  # immediate write

    def close(self):
        """Safely closes the binary log file."""
        if self.file is not None:
            with self._lock:
                if self._tlog_handle:
                    self._tlog_handle.close()
                    self._tlog_handle = None
