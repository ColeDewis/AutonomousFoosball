from src.messages.message_type import MessageType

def brick_message(flick_angle: float, twist_angle: float, speed: int, m_type: MessageType) -> str:
    """Gets the brick message for the given data.

    Args:
        flick_angle (float): flick angle to send
        twist_angle (float): twist angle to send
        speed (int): speed to run the motors at, 0-100
        m_type (MessageType): type of movement, absolute or relative
    Returns:
        str: the message to encode and send to the brick
    """
    return f"{flick_angle},{twist_angle},{speed},{m_type}".encode("UTF-8")