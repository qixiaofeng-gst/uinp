def lets_go():
    print('Yeah, let\'s go!')


if __name__ == '__main__':
    from tools.profiler import execute_with_timestamp

    execute_with_timestamp(lets_go)
