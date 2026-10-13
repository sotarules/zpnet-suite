def run():
    # Analysis tools can import the pure normalization module without starting
    # the service or requiring its PostgreSQL/transport dependencies.
    from .core import run as run_process
    return run_process()
