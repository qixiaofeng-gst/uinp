import json as __json
import os as __os
import configparser as __cp


def __get_file_path(file_name):
    return __os.path.join(__os.path.dirname(__file__), file_name)


def __get_arguments():
    import argparse
    parser = argparse.ArgumentParser(description='Parse selection')
    parser.add_argument(
        'selection', metavar='selection', type=int, default=-1,
        nargs='?', help='Code of node to switch to',
    )
    return parser.parse_args()


def __load_nodes_list():
    nodes_path = __get_file_path('surge_trojan_nodes.txt')
    parser = __cp.ConfigParser(strict=False, empty_lines_in_values=False, allow_no_value=True)
    parser.read(nodes_path)
    nodes = parser['Proxy']
    password_prefix_len = len('password=')
    parsed = []
    for key in nodes:
        parts = nodes[key].split(',')
        if 5 > len(parts):
            continue
        print('Node:', key, ', Code:', len(parsed))
        parsed.append(dict(
            remote_addr=parts[1].strip(),
            remote_port=int(parts[2].strip()),
            password=parts[3].strip()[password_prefix_len:]
        ))
    return parsed


def __load_template():
    template_path = __get_file_path('trojan_config_template.json')
    with open(template_path, 'r') as file:
        return __json.load(file)


def __update_etc(config_object: dict, remote_addr: str, remote_port: int, password: str):
    config_object.update(
        remote_addr=remote_addr,
        remote_port=remote_port,
        password=[password],
    )
    to_update = '/etc/trojan/config.json'
    with open(to_update, 'w') as file:
        __json.dump(config_object, file)
    print('Updated:', to_update)


def __restart_trojan():
    print('Tried restarting trojan:', __os.system('systemctl restart trojan'))


def __test_selection():
    print('Use the below command to test trojan:')
    print('    curl -v --socks5 127.0.0.1:1080 --connect-timeout 3 http://www.youtube.com')


def __main():
    selection = __get_arguments().selection
    print('Available nodes:')
    nodes_list = __load_nodes_list()
    if selection > -1:
        template_object = __load_template()
        __update_etc(template_object, **nodes_list[selection])
        print('Switched to', nodes_list[selection]['remote_addr'])
        __restart_trojan()
    print('Use the below command to follow the log of trojan:')
    print('    journalctl -u trojan -b -f')
    __test_selection()


if __name__ == '__main__':
    __main()
