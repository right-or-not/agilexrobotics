"""Project-specific exceptions for PiPER-X control."""


class PiperXError(RuntimeError):
    """Base exception for PiPER-X driver failures."""


class CommunicationError(PiperXError):
    """The SDK connection or feedback stream is not healthy."""


class FeedbackError(PiperXError):
    """Required feedback is missing, invalid, or stale."""


class ArmStateError(PiperXError):
    """The arm reports a state in which motion is unsafe."""


class NotEnabledError(PiperXError):
    """A motion command was attempted before this driver enabled the arm."""


class JointCommandError(PiperXError, ValueError):
    """A joint command is malformed or violates a safety limit."""
