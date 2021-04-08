# coding: utf-8
# 前置命令：cat human_recognizer-1.log | grep 'received pair' > receive_freq.txt
# 执行本脚本：python freq_parser.py


def main():
    with open('receive_freq.txt') as log_file:
        lines = log_file.readlines()
        last_ts = '17:19:52'
        last_secs = 52
        count = 0
        zeros_count = 0
        ones_count = 0
        secs_count = 0
        for line in lines:
            timestamp = line.split(' ')[2]
            timestamp = timestamp.split(',')[0]
            ts_parts = timestamp.split(':')
            secs = int(ts_parts[2])
            if timestamp == last_ts:
                count += 1
            else:
                print(timestamp, count)
                secs_count += 1
                missing_secs = secs - last_secs
                if missing_secs > 1:
                    for i in range(last_secs + 1, secs):
                        print('{}:{}:{:02d}'.format(ts_parts[0], ts_parts[1], i), 0)
                        zeros_count += 1
                        secs_count += 1
                if count == 1:
                    ones_count += 1
                count = 1
                last_ts = timestamp
                last_secs = secs
        print('Zeros: {}/{}'.format(zeros_count, secs_count))
        print('Ones: {}/{}'.format(ones_count, secs_count))


if __name__ == '__main__':
    main()
