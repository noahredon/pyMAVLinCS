import pytest
from pyMAVLinCS.mission_control_code import MCC, EMPTY

def test_mcc_creation_and_retrieval():
    """Test MCC class functionality."""
    # Create a new MCC
    mcc1 = MCC(
        value=101,
        name="TEST_SUCCESS",
        level="SUCCESS",
        description="A test success code"
    )
    
    assert mcc1.value == 101
    assert mcc1.name == "TEST_SUCCESS"
    assert mcc1.level == "SUCCESS"
    
    # Retrieve by value
    retrieved_by_val = MCC.get(101)
    assert retrieved_by_val == mcc1
    
    # Retrieve by name
    retrieved_by_name = MCC.get_name("TEST_SUCCESS")
    assert retrieved_by_name == mcc1
    
    # Check default/empty
    assert MCC.get(999) == EMPTY
    assert MCC.get_name("UNKNOWN") == EMPTY

def test_mcc_equality():
    """Test MCC equality operators."""
    mcc2 = MCC(
        value=102,
        name="TEST_INFO",
        level="INFO",
        description="A test info code"
    )
    
    # Equal to another MCC object
    assert mcc2 == MCC.get(102)
    # Equal to int
    assert mcc2 == 102
    # Equal to str
    assert mcc2 == "TEST_INFO"
    # Not equal
    assert mcc2 != 999
    assert mcc2 != "OTHER"

def test_mcc_duplicate_protection():
    """Test that creating a duplicate MCC raises an error."""
    with pytest.raises(ValueError):
        MCC(
            value=101, # Already exists from first test
            name="DUPLICATE_VAL",
            level="ERROR",
            description="Should fail"
        )
        
    with pytest.raises(ValueError):
        MCC(
            value=103,
            name="TEST_SUCCESS", # Already exists from first test
            level="ERROR",
            description="Should fail"
        )
