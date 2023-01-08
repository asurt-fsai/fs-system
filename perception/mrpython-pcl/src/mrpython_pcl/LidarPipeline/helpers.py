"""
Helper functions and classes
"""
import functools
from typing import Dict, Callable, Any
from threading import Lock


class SingletonMeta(type):
    """
    Singleton metaclass used to force a class to only have one instance of it created at a time
    """

    _instances: Dict[object, object] = {}

    def __call__(cls, *args, **kwargs):  # type: ignore[no-untyped-def]
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

    def clear(cls) -> None:
        """
        Clears the saved instance of a given class (can create a new object after calling this)
        """
        if cls in cls._instances:
            del cls._instances[cls]


def mutexLock(lock: Lock) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """
    Decorator to lock a function with a given mutex

    Parameters
    ----------
    lock : threading.Lock
        Mutex lock to fetch and release
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @functools.wraps(func)
        def newFunc(*args: Any, **kwargs: Any) -> Any:
            lock.acquire()
            val = None
            try:
                val = func(*args, **kwargs)
            finally:
                lock.release()
            return val

        return newFunc

    return decorator
