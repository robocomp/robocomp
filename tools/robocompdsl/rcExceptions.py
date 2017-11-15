
class RobocompDslException(Exception):
    """A base class for robocompdsl's exceptions."""


class InterfaceNotFound(RobocompDslException):
    """Acessing an Interface which is not found."""
