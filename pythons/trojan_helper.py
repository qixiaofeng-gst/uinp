

def __load_template():
    # trojan_config_template.json
    print(__file__)
    pass


def __load_nodes_list():
    # surge_trojan_nodes.txt
    pass


def __update_etc():
    # password: list, remote_port: int, remote_addr: string.
    # /etc/trojan/config.json
    pass


def __restart_trojan():
    # sudo systemctl restart trojan
    pass


def __test_selection():
    # curl -v --socks5 127.0.0.1:1080 --connect-timeout 3 http://www.youtube.com
    pass


def __main():
    print('Hello world')


if __name__ == '__main__':
    __main()
