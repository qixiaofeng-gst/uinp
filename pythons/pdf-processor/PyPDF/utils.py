# coding: utf-8
"""
Utility functions for PDF library.
"""
import os as _os
import io as _io
import inspect as _insp

ENCODING_UTF8 = 'utf-8'
ENCODING_UTF16 = 'utf-16'
ENCODING_UTF16BE = 'utf-16be'
DELIMITERS = b'()<>[]{}/%'

_HIGHTLIGHTEN = False


def debug(*_args):
    if not _HIGHTLIGHTEN:
        _super_print(*_args)


def stacktrace_debug():
    prefix = '>' * 2
    trace_format = '{} [{:PATH} - {:LINE}] {}'
    path_width = 0
    line_width = 0
    lines = []
    for frame in _insp.stack()[1:]:
        info = _insp.getframeinfo(frame[0])
        dirname, basename = _os.path.split(info.filename)
        dirname = _os.path.basename(dirname)
        path = _os.path.join(dirname, basename)
        lines.append([path, info.lineno, info.function, info.code_context])
        path_width = len(path) if len(path) > path_width else path_width
        line_width = len(str(info.lineno)) if len(str(info.lineno)) > line_width else line_width
    trace_format = trace_format.replace('PATH', str(path_width))
    trace_format = trace_format.replace('LINE', str(line_width))
    for path, line, func, context in lines:
        print(trace_format.format(
            prefix, path, line, func,
        ))
        for code in context:
            if code.endswith('\n'):
                code = code[:-1]
            print('{} {}'.format(prefix, code))


def hightlight_debug(*_args):
    global _HIGHTLIGHTEN
    _HIGHTLIGHTEN = True
    print('>' * 32, 'hightlighted', '<' * 32)
    _super_print(*_args)


def _super_print(*args):
    info = _insp.getframeinfo(_insp.stack()[2][0])
    dirname, basename = _os.path.split(info.filename)
    dirname = _os.path.basename(dirname)
    print('[{}] {} [{}]'.format(_os.path.join(dirname, basename), info.function, info.lineno), *args)


def s2b(s: str):
    return bytes(s, ENCODING_UTF8)


def read_hex_bytes_from(stream):
    stream.read(1)
    txt = b''
    x = b''
    while True:
        tok = read_non_whitespace(stream)
        if tok == b'>':
            break
        x += tok
        if len(x) == 2:
            txt += s2b(chr(int(x, base=16)))
            x = b''
    if len(x) == 1:
        x += b'0'
    if len(x) == 2:
        txt += s2b(chr(int(x, base=16)))
    return txt


def read_bytes_from(stream):
    _tok = stream.read(1)
    parens = 1
    txt = b''
    while True:
        tok = stream.read(1)
        if tok in b'(':
            parens += 1
        elif tok in b')':
            parens -= 1
            if parens == 0:
                break
        elif tok in b'\\':
            tok = stream.read(1)
            if tok in b'n':
                tok = b'\n'
            elif tok in b'r':
                tok = b'\r'
            elif tok in b't':
                tok = b'\t'
            elif tok in b'b':
                tok = b'\b'
            elif tok in b'f':
                tok = b'\f'
            elif tok in b'()\\':
                pass
            elif tok.isdigit():
                # "The number ddd may consist of one, two, or three
                # octal digits; high-order overflow shall be ignored.
                # Three octal digits shall be used, with leading zeros
                # as needed, if the next character of the string is also
                # a digit." (PDF reference 7.3.4.2, p 16)
                for _i in range(2):
                    ntok = stream.read(1)
                    if ntok.isdigit():
                        tok += ntok
                    else:
                        break
                tok = s2b(chr(int(tok, base=8)))
            elif tok in b'\n\r':
                # This case is  hit when a backslash followed by a line
                # break occurs.  If it's a multi-char EOL, consume the
                # second character:
                tok = stream.read(1)
                if tok not in b'\n\r':
                    stream.seek(-1, _io.SEEK_CUR)
                # Then don't add anything to the actual string, since this
                # line break was escaped:
                tok = b''
            else:
                raise PdfReadError("Unexpected escaped string")
        txt += tok
    return txt


def read_until_whitespace(stream, maxchars=None):
    txt = b''
    while True:
        tok = stream.read(1)
        if tok.isspace() or not tok:
            break
        txt += tok
        if len(txt) == maxchars:
            break
    return txt


def read_non_whitespace(stream: _io.BufferedReader, seek_back: bool = False):
    tok = b' '
    while tok in b'\n\r\t ' and len(tok) > 0:
        tok = stream.read(1)
    if seek_back:
        stream.seek(-1, _io.SEEK_CUR)
    return tok


def seek_token(stream):
    return read_non_whitespace(stream, True)


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


def encode_pdf_doc_encoding(unicode_string):
    retval = ''
    for c in unicode_string:
        try:
            retval += chr(_PDF_DOC_ENCODING_REVERSED[c])
        except KeyError:
            raise UnicodeEncodeError("pdfdocencoding", c, -1, -1, "does not exist in translation table")
    return retval


def decode_pdf_doc_encoding(byte_array: bytes):
    retval = u''
    for b in byte_array:
        c = _PDF_DOC_ENCODING[b]
        if c == u'\u0000':
            raise UnicodeDecodeError("pdfdocencoding", bytes([b]), -1, -1, "does not exist in translation table")
        retval += c
    return retval


_PDF_DOC_ENCODING = (
    u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000',
    u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000',
    u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000', u'\u0000',
    u'\u02d8', u'\u02c7', u'\u02c6', u'\u02d9', u'\u02dd', u'\u02db', u'\u02da', u'\u02dc',
    u'\u0020', u'\u0021', u'\u0022', u'\u0023', u'\u0024', u'\u0025', u'\u0026', u'\u0027',
    u'\u0028', u'\u0029', u'\u002a', u'\u002b', u'\u002c', u'\u002d', u'\u002e', u'\u002f',
    u'\u0030', u'\u0031', u'\u0032', u'\u0033', u'\u0034', u'\u0035', u'\u0036', u'\u0037',
    u'\u0038', u'\u0039', u'\u003a', u'\u003b', u'\u003c', u'\u003d', u'\u003e', u'\u003f',
    u'\u0040', u'\u0041', u'\u0042', u'\u0043', u'\u0044', u'\u0045', u'\u0046', u'\u0047',
    u'\u0048', u'\u0049', u'\u004a', u'\u004b', u'\u004c', u'\u004d', u'\u004e', u'\u004f',
    u'\u0050', u'\u0051', u'\u0052', u'\u0053', u'\u0054', u'\u0055', u'\u0056', u'\u0057',
    u'\u0058', u'\u0059', u'\u005a', u'\u005b', u'\u005c', u'\u005d', u'\u005e', u'\u005f',
    u'\u0060', u'\u0061', u'\u0062', u'\u0063', u'\u0064', u'\u0065', u'\u0066', u'\u0067',
    u'\u0068', u'\u0069', u'\u006a', u'\u006b', u'\u006c', u'\u006d', u'\u006e', u'\u006f',
    u'\u0070', u'\u0071', u'\u0072', u'\u0073', u'\u0074', u'\u0075', u'\u0076', u'\u0077',
    u'\u0078', u'\u0079', u'\u007a', u'\u007b', u'\u007c', u'\u007d', u'\u007e', u'\u0000',
    u'\u2022', u'\u2020', u'\u2021', u'\u2026', u'\u2014', u'\u2013', u'\u0192', u'\u2044',
    u'\u2039', u'\u203a', u'\u2212', u'\u2030', u'\u201e', u'\u201c', u'\u201d', u'\u2018',
    u'\u2019', u'\u201a', u'\u2122', u'\ufb01', u'\ufb02', u'\u0141', u'\u0152', u'\u0160',
    u'\u0178', u'\u017d', u'\u0131', u'\u0142', u'\u0153', u'\u0161', u'\u017e', u'\u0000',
    u'\u20ac', u'\u00a1', u'\u00a2', u'\u00a3', u'\u00a4', u'\u00a5', u'\u00a6', u'\u00a7',
    u'\u00a8', u'\u00a9', u'\u00aa', u'\u00ab', u'\u00ac', u'\u0000', u'\u00ae', u'\u00af',
    u'\u00b0', u'\u00b1', u'\u00b2', u'\u00b3', u'\u00b4', u'\u00b5', u'\u00b6', u'\u00b7',
    u'\u00b8', u'\u00b9', u'\u00ba', u'\u00bb', u'\u00bc', u'\u00bd', u'\u00be', u'\u00bf',
    u'\u00c0', u'\u00c1', u'\u00c2', u'\u00c3', u'\u00c4', u'\u00c5', u'\u00c6', u'\u00c7',
    u'\u00c8', u'\u00c9', u'\u00ca', u'\u00cb', u'\u00cc', u'\u00cd', u'\u00ce', u'\u00cf',
    u'\u00d0', u'\u00d1', u'\u00d2', u'\u00d3', u'\u00d4', u'\u00d5', u'\u00d6', u'\u00d7',
    u'\u00d8', u'\u00d9', u'\u00da', u'\u00db', u'\u00dc', u'\u00dd', u'\u00de', u'\u00df',
    u'\u00e0', u'\u00e1', u'\u00e2', u'\u00e3', u'\u00e4', u'\u00e5', u'\u00e6', u'\u00e7',
    u'\u00e8', u'\u00e9', u'\u00ea', u'\u00eb', u'\u00ec', u'\u00ed', u'\u00ee', u'\u00ef',
    u'\u00f0', u'\u00f1', u'\u00f2', u'\u00f3', u'\u00f4', u'\u00f5', u'\u00f6', u'\u00f7',
    u'\u00f8', u'\u00f9', u'\u00fa', u'\u00fb', u'\u00fc', u'\u00fd', u'\u00fe', u'\u00ff'
)

assert len(_PDF_DOC_ENCODING) == 256

_PDF_DOC_ENCODING_REVERSED = {}
for __idx in range(256):
    char = _PDF_DOC_ENCODING[__idx]
    if char == u"\u0000":
        continue
    assert char not in _PDF_DOC_ENCODING_REVERSED
    _PDF_DOC_ENCODING_REVERSED[char] = __idx
if __name__ == "__main__":
    # test RC4
    out = rc4_encrypt("Key", "Plaintext")
    print(repr(out))
    pt = rc4_encrypt("Key", out)
    print(repr(pt))
