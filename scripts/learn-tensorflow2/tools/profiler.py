def executeWithTimestamp(cbFunction):
    from datetime import datetime
    startTimestamp = datetime.now().timestamp()
    cbFunction()
    print('Cost time of [{}]: {:.6f}s'.format(
        cbFunction.__name__,
        datetime.now().timestamp() - startTimestamp
    ))
