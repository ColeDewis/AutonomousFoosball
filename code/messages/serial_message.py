def serial_message(motor_id: int, target: int):
    """Initialize the message

    Args:
        motor_id (int): id of the motor; 0 to reset
        target (int): target position of the motor
    Returns:
        bytearray: the bytearray packet message.
    """
    # to speed up serial, we send the message in 4 bytes:
    # byte 1 - stepper id
    # byte 2 - movement direction (0 or 1) - 0 is positive, 1 is negative
    # byte 3/4 - target position 
    packet = bytearray()
    packet.append(motor_id)
    signbyte = 0 if target >= 0 else 1
    packet.append(signbyte)
    targetbytes = (abs(target)).to_bytes(2)
    packet += targetbytes
    return packet