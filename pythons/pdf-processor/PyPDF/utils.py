# coding: utf-8
"""
Utility functions for PDF library.
"""


# ENABLE_PSYCO = False
# if ENABLE_PSYCO:
#    try:
#        import psyco
#    except ImportError:
#        ENABLE_PSYCO = False
#
# if not ENABLE_PSYCO:
#    class psyco:
#        def proxy(func):
#            return func
#        proxy = staticmethod(proxy)
def read_until_whitespace(stream, maxchars=None):
    txt = ""
    while True:
        tok = stream.read(1)
        if tok.isspace() or not tok:
            break
        txt += tok
        if len(txt) == maxchars:
            break
    return txt


def read_non_whitespace(stream):
    tok = ' '
    while tok == '\n' or tok == '\r' or tok == ' ' or tok == '\t':
        tok = stream.read(1)
    return tok


class ConvertFunctionsToVirtualList(object):
    def __init__(self, length_function, get_function):
        self.lengthFunction = length_function
        self.getFunction = get_function

    def __len__(self):
        return self.lengthFunction()

    def __getitem__(self, index):
        if not isinstance(index, int):
            raise TypeError("sequence indices must be integers")
        len_self = len(self)
        if index < 0:
            # support negative indexes
            index = len_self + index
        if index < 0 or index >= len_self:
            raise IndexError("sequence index out of range")
        return self.getFunction(index)


def rc4_encrypt(key, plaintext):
    s = [i for i in range(256)]
    j = 0
    for i in range(256):
        j = (j + s[i] + ord(key[i % len(key)])) % 256
        s[i], s[j] = s[j], s[i]
    i, j = 0, 0
    retval = ""
    for x in range(len(plaintext)):
        i = (i + 1) % 256
        j = (j + s[i]) % 256
        s[i], s[j] = s[j], s[i]
        t = s[(s[i] + s[j]) % 256]
        retval += chr(ord(plaintext[x]) ^ t)
    return retval


def matrix_multiply(a, b):
    return [[sum([float(i) * float(j)
                  for i, j in zip(row, col)]
                 ) for col in zip(*b)]
            for row in a]


class PyPdfError(Exception):
    pass


class PdfReadError(PyPdfError):
    pass


class PageSizeNotDefinedError(PyPdfError):
    pass


BYTES_ENCODING = 'utf-8'


if __name__ == "__main__":
    # test RC4
    out = rc4_encrypt("Key", "Plaintext")
    print(repr(out))
    pt = rc4_encrypt("Key", out)
    print(repr(pt))
