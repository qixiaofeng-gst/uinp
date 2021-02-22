# vim: sw=4:expandtab:foldmethod=marker
#
# Copyright (c) 2006, Mathieu Fenniak
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# * The name of the author may not be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Implementation of stream filters for PDF.
"""
__author__ = "Mathieu Fenniak"
__author_email__ = "biziqe@mathieu.fenniak.net"

from io import StringIO
from pdf.utils import PdfReadError
from zlib import decompress, compress


class FlateDecode(object):
    @staticmethod
    def decode(data, decode_parms):
        data = decompress(data)
        predictor = 1
        if decode_parms:
            predictor = decode_parms.get("/Predictor", 1)
        # predictor 1 == no predictor
        if predictor != 1:
            columns = decode_parms["/Columns"]
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
    def decode(data):
        retval = ""
        char = ""
        x = 0
        while True:
            c = data[x]
            if c == ">":
                break
            elif c.isspace():
                x += 1
                continue
            char += c
            if len(char) == 2:
                retval += chr(int(char, base=16))
                char = ""
            x += 1
        assert char == ""
        return retval

    decode = staticmethod(decode)


class ASCII85Decode(object):
    def decode(data):
        retval = ""
        group = []
        x = 0
        hit_eod = False
        # remove all whitespace from data
        data = [y for y in data if not (y in ' \n\r\t')]
        while not hit_eod:
            c = data[x]
            if len(retval) == 0 and c == "<" and data[x + 1] == "~":
                x += 2
                continue
            # elif c.isspace():
            #    x += 1
            #    continue
            elif c == 'z':
                assert len(group) == 0
                retval += '\x00\x00\x00\x00'
                continue
            elif c == "~" and data[x + 1] == ">":
                if len(group) != 0:
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

    decode = staticmethod(decode)


def decode_stream_data(stream):
    from pdf.generic import NameObject
    filters = stream.get("/Filter", ())
    if len(filters) and not isinstance(filters[0], NameObject):
        # we have a single filter instance
        filters = (filters,)
    data = stream._data
    for filterType in filters:
        if filterType == "/FlateDecode":
            data = FlateDecode.decode(data, stream.get("/DecodeParms"))
        elif filterType == "/ASCIIHexDecode":
            data = ASCIIHexDecode.decode(data)
        elif filterType == "/ASCII85Decode":
            data = ASCII85Decode.decode(data)
        elif filterType == "/Crypt":
            decode_params = stream.get("/DecodeParams", {})
            if "/Name" not in decode_params and "/Type" not in decode_params:
                pass
            else:
                raise NotImplementedError("/Crypt filter with /Name or /Type not supported yet")
        else:
            # unsupported filter
            raise NotImplementedError("unsupported filter %s" % filterType)
    return data


if __name__ == "__main__":
    assert "abc" == ASCIIHexDecode.decode('61\n626\n3>')

    ascii85Test = """
     <~9jqo^BlbD-BleB1DJ+*+F(f,q/0JhKF<GL>Cj@.4Gp$d7F!,L7@<6@)/0JDEF<G%<+EV:2F!,
     O<DJ+*.@<*K0@<6L(Df-\\0Ec5e;DffZ(EZee.Bl.9pF"AGXBPCsi+DGm>@3BB/F*&OCAfu2/AKY
     i(DIb:@FD,*)+C]U=@3BN#EcYf8ATD3s@q?d$AftVqCh[NqF<G:8+EV:.+Cf>-FD5W8ARlolDIa
     l(DId<j@<?3r@:F%a+D58'ATD4$Bl@l3De:,-DJs`8ARoFb/0JMK@qB4^F!,R<AKZ&-DfTqBG%G
     >uD.RTpAKYo'+CT/5+Cei#DII?(E,9)oF*2M7/c~>
    """
    ascii85_originalText = ('Man is distinguished, not only by his reason, '
                            'but by this singular passion from other animals, '
                            'which is a lust of the mind, that by a perseverance of delight in the continued '
                            'and indefatigable generation of knowledge, '
                            'exceeds the short vehemence of any carnal pleasure.')
    assert ASCII85Decode.decode(ascii85Test) == ascii85_originalText
