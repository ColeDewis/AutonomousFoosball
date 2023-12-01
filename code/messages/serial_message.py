def serial_message(motor_id: int, speed: int, target: int) -> bytearray:
    """Initialize the message

    Args:
        motor_id (int): id of the motor; 0 to reset
        speed (int): speed of the motor, to be multiplied by 100.
        target (int): target position of the motor
    Returns:
        bytearray: the bytearray packet message.
    """
    # to speed up serial, we send the message in 4 bytes:
    # byte 1 - stepper id
    # byte 2 - bit 0..6 is speed, to be multiplied by 100. bit 7 if target is negative
    # byte 3/4 - target position in steps
    packet = bytearray()
    packet.append(motor_id)
    speed_sign_byte = 0
    if target < 0:
        speed_sign_byte |= (1 << 7)
    speed_sign_byte |= speed
    packet.append(speed_sign_byte)
    targetbytes = (abs(target)).to_bytes(2)
    packet += targetbytes
    return packet

print(serial_message(2, 10, 1500))