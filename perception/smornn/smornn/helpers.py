"""
Helper functions and classes
"""
import functools
from typing import Callable, Any
from threading import Lock


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
