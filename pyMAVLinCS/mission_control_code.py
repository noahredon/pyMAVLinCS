# pyMAVLinCS/mission_control_code.py
# Copyright (C) 2025 Noah Redon
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

class MCC:
    """
    Class representing an MCC (Mission Control Code) with various properties, levels, and descriptions.
    Allows categorization of MCCs based on their value and associates descriptions with them.
    """

    # Static dictionary to store MCC instances by value
    __dict: dict[int, 'MCC'] = {}

    # Static dictionary to store MCC instances by name
    __dict_name: dict[str, 'MCC'] = {}

    def __init__(self,
                 value: int,
                 name: str,
                 level: str,
                 description: str):
        """
        Initializes a new MCC object.

        Args:
            value (int): The unique numeric value associated with the MCC.
            name (str): The name of the MCC.
            level (str): Type of MCC (e.g., SUCCESS).
            description (str): A short description of the MCC.
        """
        self.value = value
        self.name = name
        self.level = level
        self.description = f"[{level}][{name}] {description}"

        # Check if the MCC already exists in the static dictionary
        if self.value in MCC.__dict:
            raise ValueError("Already existing MCC")  # Prevent duplicates

        # Check if the MCC already exists in the static dictionary
        if self.name in MCC.__dict_name:
            raise ValueError("Already existing MCC")  # Prevent duplicates

        # Add the MCC to the static dictionaries
        MCC.__dict[self.value] = self
        MCC.__dict_name[self.name] = self

    def __repr__(self) -> str:
        """
        Returns a concise textual representation of the MCC object.
        Includes the level, value, and a short description.
        """
        return self.description

    def __int__(self) -> int:
        """
        Allows conversion of an MCC object to an integer corresponding to its value.
        """
        return self.value

    def __eq__(self, other) -> bool:
        if isinstance(other, MCC):
            return self.value == other.value
        elif isinstance(other, int):
            return self.value == other
        elif isinstance(other, str):
            return self.name == other
        return False

    def __hash__(self):
        return hash(self.value)

    @staticmethod
    def get(value: int) -> 'MCC':
        """
        Retrieves an MCC object by its numeric value.
        Returns EMPTY if no MCC exists with this value.
        """
        return MCC.__dict.get(value, EMPTY)

    @staticmethod
    def get_name(name: str) -> 'MCC':
        """
        Retrieves an MCC object by its name.
        Returns EMPTY if no MCC exists with this name.
        """
        return MCC.__dict_name.get(name, EMPTY)

EMPTY = MCC(
    value=0,
    name="EMPTY",
    level="EMPTY",
    description="empty"
)
