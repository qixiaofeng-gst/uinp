WHITE_SPACE_CHARACTERS = [
    b'\x00',  # \000, NUL, null
    b'\x09',  # \011, HT, horizontal tab
    b'\x0A',  # \012, LF, line feed
    b'\x0C',  # \014, FF, form feed
    b'\x0D',  # \015, CR, carriage return
    b'\x20',  # \040, SP, space
]  # CR and LF are new line characters. They are treated as EOL(end-of-line) markers.
assert b'\x09' == b'\11'
assert b'\x09' == b'\011'
assert not b'\x09' == b'\0110'
assert b'\x0A' == b'\n'
assert b'\x0D' == b'\r'
# The b'\r\n' shall be treated as one EOL.
DELIMITER_CHARACTERS = [
    b'\x28',  # \050, (, left parenthesis
    b'\x29',  # \051, (, right parenthesis
    b'\x3C',  # \060, (, less-than sign
    b'\x3E',  # \062, (, greater-than sign
    b'\x5B',  # \133, (, left square bracket
    b'\x5D',  # \135, (, right square bracket
    b'\x7B',  # \173, (, left curly bracket
    b'\x7D',  # \175, (, right curly bracket
    b'\x2F',  # \057, (, solidus
    b'\x25',  # \045, (, percent sign
]
assert b'\x28' == b'\050'
