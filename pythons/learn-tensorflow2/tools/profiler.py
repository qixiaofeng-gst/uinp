def execute_with_timestamp(cb_function):
    from datetime import datetime
    start_timestamp = datetime.now().timestamp()
    cb_function()
    print('Cost time of [{}]: {:.6f}s'.format(
        cb_function.__name__,
        datetime.now().timestamp() - start_timestamp
    ))
