import json as __json
import os as __os
import configparser as __cp


def __get_file(file_name):
    return __os.path.join(__os.path.dirname(__file__), file_name)


def __load_template():
    template_path = __get_file('trojan_config_template.json')
    with open(template_path, 'r') as file:
        return __json.load(file)


def __load_nodes_list():
    # surge_trojan_nodes.txt
    pass


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
    # curl -v --socks5 127.0.0.1:1080 --connect-timeout 3 http://www.youtube.com
    pass


def __main():
    template_object = __load_template()
    __update_etc(template_object, 'jpa1.v2dot.online', 443, '53677FFF-F237-93D8-15A3-0991112C4233')
    # __restart_trojan()
    print('Use `journalctl -u trojan -b -f` to follow the log of trojan.')


if __name__ == '__main__':
    __main()
