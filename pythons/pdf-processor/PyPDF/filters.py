# coding: utf-8
"""
Implementation of stream filters for PDF.
"""
__author__ = "Mathieu Fenniak"
__author_email__ = "biziqe@mathieu.fenniak.net"

from io import StringIO
from PyPDF.utils import PdfReadError
from zlib import decompress, compress


class FlateDecode(object):
    @staticmethod
    def decode(data, decode_parms):
        data = decompress(data)
        predictor = 1
        if decode_parms:
            predictor = decode_parms.get(b'/Predictor', 1)
        # predictor 1 == no predictor
        if predictor != 1:
            columns = decode_parms[b'/Columns']
            # PNG prediction:
            if 10 <= predictor <= 15:
                output = StringIO()
                # PNG prediction can vary from row to row
                rowlength = columns + 1
                assert len(data) % rowlength == 0
                prev_rowdata = (0,) * rowlength
                for row in range(len(data) / rowlength):
                    rowdata = [ord(x) for x in data[(row * rowlength):((row + 1) * rowlength)]]
                    filter_byte = rowdata[0]
                    if filter_byte == 0:
                        pass
                    elif filter_byte == 1:
                        for i in range(2, rowlength):
                            rowdata[i] = (rowdata[i] + rowdata[i - 1]) % 256
                    elif filter_byte == 2:
                        for i in range(1, rowlength):
                            rowdata[i] = (rowdata[i] + prev_rowdata[i]) % 256
                    else:
                        # unsupported PNG filter
                        raise PdfReadError("Unsupported PNG filter %r" % filter_byte)
                    prev_rowdata = rowdata
                    output.write(''.join([chr(x) for x in rowdata[1:]]))
                data = output.getvalue()
            else:
                # unsupported predictor
                raise PdfReadError("Unsupported flatedecode predictor %r" % predictor)
        return data

    @staticmethod
    def encode(data):
        return compress(data)


class ASCIIHexDecode(object):
    @staticmethod
    def decode(data):
        retval = ""
        char = b''
        x = 0
        while True:
            c = data[x:x+1]
            if c in b'>':
                break
            elif c.isspace():
                x += 1
                continue
            char += c
            if len(char) == 2:
                retval += chr(int(char, base=16))
                char = b''
            x += 1
        assert char == b''
        return retval


class ASCII85Decode(object):
    @staticmethod
    def decode(data: bytes):
        retval = ""
        group = []
        x = 0
        hit_eod = False
        # remove all whitespace from data
        data = bytes(y for y in data if not (y in b' \n\r\t'))
        while not hit_eod:
            c = data[x:x+1]
            if len(retval) == 0 and c in b'<' and data[x+1:x+2] in b'~':
                x += 2
                continue
            elif c.isspace():
                x += 1
                continue
            elif c in b'z':
                assert len(group) == 0
                retval += '\x00\x00\x00\x00'
                continue
            elif c in b'~' and data[x+1:x+2] in b'>':
                if len(group) > 0:
                    # cannot have a final group of just 1 char
                    assert len(group) > 1
                    cnt = len(group) - 1
                    group += [85, 85, 85]
                    hit_eod = cnt
                else:
                    break
            else:
                c = ord(c) - 33
                assert 0 <= c < 85
                group += [c]
            if len(group) >= 5:
                b = group[0] * (85 ** 4) + \
                    group[1] * (85 ** 3) + \
                    group[2] * (85 ** 2) + \
                    group[3] * 85 + \
                    group[4]
                assert b < (2 ** 32 - 1)
                c4 = chr((b >> 0) % 256)
                c3 = chr((b >> 8) % 256)
                c2 = chr((b >> 16) % 256)
                c1 = chr(b >> 24)
                retval += (c1 + c2 + c3 + c4)
                if hit_eod:
                    retval = retval[:-4 + hit_eod]
                group = []
            x += 1
        return retval


def decode_stream_data(stream):
    from PyPDF.generic import NameObject
    filters = stream.get(b'/Filter', ())
    if len(filters) and not isinstance(filters[0], NameObject):
        # we have a single filter instance
        filters = (filters,)
    data = stream.bytes_data
    for filterType in filters:
        if filterType == b'/FlateDecode':
            data = FlateDecode.decode(data, stream.get(b'/DecodeParms'))
        elif filterType == b'/ASCIIHexDecode':
            data = ASCIIHexDecode.decode(data)
        elif filterType == b'/ASCII85Decode':
            data = ASCII85Decode.decode(data)
        elif filterType == b'/Crypt':
            decode_params = stream.get(b'/DecodeParams', {})
            if b'/Name' not in decode_params and b'/Type' not in decode_params:
                pass
            else:
                raise NotImplementedError("/Crypt filter with /Name or /Type not supported yet")
        else:
            raise NotImplementedError("unsupported filter %s" % filterType)
    return data


if __name__ == "__main__":
    assert "abc" == ASCIIHexDecode.decode(b'61\n626\n3>')

    ascii85Test = b'''
     <~9jqo^BlbD-BleB1DJ+*+F(f,q/0JhKF<GL>Cj@.4Gp$d7F!,L7@<6@)/0JDEF<G%<+EV:2F!,
     O<DJ+*.@<*K0@<6L(Df-\\0Ec5e;DffZ(EZee.Bl.9pF"AGXBPCsi+DGm>@3BB/F*&OCAfu2/AKY
     i(DIb:@FD,*)+C]U=@3BN#EcYf8ATD3s@q?d$AftVqCh[NqF<G:8+EV:.+Cf>-FD5W8ARlolDIa
     l(DId<j@<?3r@:F%a+D58'ATD4$Bl@l3De:,-DJs`8ARoFb/0JMK@qB4^F!,R<AKZ&-DfTqBG%G
     >uD.RTpAKYo'+CT/5+Cei#DII?(E,9)oF*2M7/c~>
    '''
    ascii85_originalText = ("Man is distinguished, not only by his reason, "
                            "but by this singular passion from other animals, "
                            "which is a lust of the mind, that by a perseverance of delight in the continued "
                            "and indefatigable generation of knowledge, "
                            "exceeds the short vehemence of any carnal pleasure.")
    assert ASCII85Decode.decode(ascii85Test) == ascii85_originalText
