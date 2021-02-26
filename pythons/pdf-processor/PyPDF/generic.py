# coding: utf-8
"""
Implementation of generic PDF objects (dictionary, number, string, and so on)
"""

import io
import re
from PyPDF.utils import read_non_whitespace, seek_token, rc4_encrypt
import PyPDF.filters as filters
import PyPDF.utils as utils
import decimal
import codecs


def read_object(stream: io.BufferedReader, pdf_reader):
    tok = stream.read(1)
    stream.seek(-1, io.SEEK_CUR)  # reset to start
    if tok in b'tf':
        # boolean object
        return BooleanObject.read_from_stream(stream)
    elif tok in b'(':
        # string object
        return read_string_from_stream(stream)
    elif tok in b'/':
        # name object
        return NameObject.read_from_stream(stream)
    elif tok in b'[':
        # array object
        return ArrayObject.read_from_stream(stream, pdf_reader)
    elif tok in b'n':
        # null object
        return NullObject.read_from_stream(stream)
    elif tok in b'<':
        # hexadecimal string OR dictionary
        peek = stream.read(2)
        stream.seek(-2, io.SEEK_CUR)  # reset to start
        if peek == b'<<':
            return DictionaryObject.read_from_stream(stream, pdf_reader)
        else:
            return read_hex_string_from_stream(stream)
    elif tok in b'%':
        # comment
        while tok not in b'\r\n':
            tok = stream.read(1)
        seek_token(stream)
        return read_object(stream, pdf_reader)
    else:
        # number object OR indirect reference
        if tok in b'+-':
            # number
            return NumberObject.read_from_stream(stream)
        peek = stream.read(20)
        stream.seek(-len(peek), io.SEEK_CUR)  # reset to start
        if re.match(br'(\d+)\s(\d+)\sR[^a-zA-Z]', peek) is not None:
            return IndirectObject.read_from_stream(stream, pdf_reader)
        else:
            return NumberObject.read_from_stream(stream)


class PdfObject(object):
    def get_object(self):
        """Resolves indirect references."""
        return self


class NullObject(PdfObject):
    @staticmethod
    def write_to_stream(stream):
        stream.write(b'null')

    @staticmethod
    def read_from_stream(stream):
        nulltxt = stream.read(4)
        if not nulltxt == b'null':
            raise utils.PdfReadError("error reading null object")
        return NullObject()


class BooleanObject(PdfObject):
    def __init__(self, value):
        self.value = value

    def write_to_stream(self, stream):
        if self.value:
            stream.write(b'true')
        else:
            stream.write(b'false')

    @staticmethod
    def read_from_stream(stream):
        utils.debug(stream.read(10))
        stream.seek(-10, io.SEEK_CUR)
        word = stream.read(4)
        if word == b'true':
            return BooleanObject(True)
        elif word == b'fals':
            stream.read(1)
            return BooleanObject(False)
        assert False


class ArrayObject(list, PdfObject):
    def write_to_stream(self, stream, encryption_key):
        stream.write(b'[')
        for data in self:
            stream.write(b' ')
            data.write_to_stream(stream, encryption_key)
        stream.write(b' ]')

    @staticmethod
    def read_from_stream(stream, pdf):
        arr = ArrayObject()
        tmp = stream.read(1)
        if tmp not in b'[':
            raise utils.PdfReadError("error reading array")
        while True:
            # skip leading whitespace
            tok = stream.read(1)
            while tok.isspace():
                tok = stream.read(1)
            stream.seek(-1, io.SEEK_CUR)
            # check for array ending
            peekahead = stream.read(1)
            if peekahead in b']':
                break
            stream.seek(-1, io.SEEK_CUR)
            # read and append obj
            arr.append(read_object(stream, pdf))
        return arr


class IndirectObject(PdfObject):
    def __init__(self, idnum, generation, pdf):
        self.idnum = idnum
        self.generation = generation
        self.pdf = pdf

    def get_object(self):
        return self.pdf.get_object(self).get_object()

    def __repr__(self):
        return "IndirectObject(%r, %r, %s)" % (self.idnum, self.generation, type(self.pdf))

    def __eq__(self, other):
        return (other is not None and
                isinstance(other, IndirectObject) and
                self.idnum == other.idnum and
                self.generation == other.generation and
                self.pdf is other.pdf)

    def __ne__(self, other):
        return not self.__eq__(other)

    def write_to_stream(self, stream):
        stream.write(bytes('%s %s R' % (self.idnum, self.generation), utils.ENCODING_UTF8))

    @staticmethod
    def read_from_stream(stream, pdf):
        idnum = b''
        while True:
            tok = stream.read(1)
            if tok.isspace():
                break
            idnum += tok
        generation = b''
        while True:
            tok = stream.read(1)
            if tok.isspace():
                break
            generation += tok
        r = stream.read(1)
        if r not in b'R':
            raise utils.PdfReadError("error reading indirect object reference")
        return IndirectObject(int(idnum), int(generation), pdf)


class FloatObject(decimal.Decimal, PdfObject):
    def __new__(cls, value='0', context=None):
        # FIXME Warning here.
        # noinspection PyTypeChecker
        return decimal.Decimal.__new__(cls, value, context)

    def __repr__(self):
        if self == self.to_integral():
            return bytes(str(self.quantize(decimal.Decimal(1))), utils.ENCODING_UTF8)
        else:
            # XXX: this adds useless extraneous zeros.
            return bytes("%.5f" % self, utils.ENCODING_UTF8)

    def write_to_stream(self, stream):
        stream.write(bytes(repr(self), utils.ENCODING_UTF8))


class NumberObject(int, PdfObject):
    def __init__(self, value):
        int.__init__(value)

    def write_to_stream(self, stream):
        stream.write(bytes(repr(self), utils.ENCODING_UTF8))

    @staticmethod
    def read_from_stream(stream):
        name = b''
        while True:
            tok = stream.read(1)
            if not (tok in b'+-.' or tok.isdigit()):
                stream.seek(-1, io.SEEK_CUR)
                break
            name += tok
        utils.debug(name)
        if name.find(b'.') > -1:
            return FloatObject(name.decode(utils.ENCODING_UTF8))
        else:
            return NumberObject(name)


##
# Represents a string object where the text encoding could not be determined.
# This occurs quite often, as the PDF spec doesn't provide an alternate way to
# represent strings -- for example, the encryption data stored in files (like
# /O) is clearly not text, but is still stored in a "String" object.
class ByteStringObject(str, PdfObject):
    ##
    # For compatibility with TextStringObject.original_bytes.  This method
    # returns self.
    original_bytes = property(lambda self: self)

    def write_to_stream(self, stream, encryption_key):
        bytearr = self
        if encryption_key:
            bytearr = rc4_encrypt(encryption_key, bytearr)
        stream.write(b'<')
        stream.write(bytearr.encode("hex"))
        stream.write(b'>')


##
# Represents a string object that has been decoded into a real unicode string.
# If read from a PDF document, this string appeared to match the
# PDFDocEncoding, or contained a UTF-16BE BOM mark to cause UTF-16 decoding to
# occur.
# FIXME class TextStringObject(unicode, PdfObject):
class TextStringObject(str, PdfObject):
    autodetect_pdfdocencoding = False
    autodetect_utf16 = False

    ##
    # It is occasionally possible that a text string object gets created where
    # a byte string object was expected due to the autodetection mechanism --
    # if that occurs, this "original_bytes" property can be used to
    # back-calculate what the original encoded bytes were.
    original_bytes = property(lambda self: self.get_original_bytes())

    def get_original_bytes(self):
        # We're a text string object, but the library is trying to get our raw
        # bytes.  This can happen if we auto-detected this string as text, but
        # we were wrong.  It's pretty common.  Return the original bytes that
        # would have been used to create this object, based upon the autodetect
        # method.
        if self.autodetect_utf16:
            return codecs.BOM_UTF16_BE + self.encode(utils.ENCODING_UTF16BE)
        elif self.autodetect_pdfdocencoding:
            return encode_pdfdocencoding(self)
        else:
            raise Exception("no information about original bytes")

    def write_to_stream(self, stream, encryption_key):
        # Try to write the string out as a PDFDocEncoding encoded string.  It's
        # nicer to look at in the PDF file.  Sadly, we take a performance hit
        # here for trying...
        try:
            bytearr = encode_pdfdocencoding(self)
        except UnicodeEncodeError:
            bytearr = codecs.BOM_UTF16_BE + self.encode(utils.ENCODING_UTF16BE)
        if encryption_key:
            bytearr = rc4_encrypt(encryption_key, bytearr)
            obj = ByteStringObject(bytearr)
            obj.write_to_stream(stream, None)
        else:
            stream.write("(")
            for c in bytearr:
                if not c.isalnum() and c != ' ':
                    stream.write("\\%03o" % ord(c))
                else:
                    stream.write(c)
            stream.write(")")


class NameObject(bytes, PdfObject):
    def __init__(self, data):
        bytes.__init__(data)

    def write_to_stream(self, stream):
        stream.write(self)

    @staticmethod
    def read_from_stream(stream):
        name = stream.read(1)
        if name not in b'/':
            raise utils.PdfReadError("name read error")
        while True:
            tok = stream.read(1)
            if tok.isspace() or tok in utils.DELIMITERS:
                stream.seek(-1, io.SEEK_CUR)
                break
            name += tok
        return NameObject(name)


class DictionaryObject(dict, PdfObject):
    def __init__(self, *args, **kwargs):
        super(DictionaryObject, self).__init__()
        if len(args) == 0:
            self.update(kwargs)
        elif len(args) == 1:
            arr = args[0]
            # If we're passed a list/tuple, make a dict out of it
            if not hasattr(arr, "iteritems"):
                newarr = {}
                for k, v in arr:
                    newarr[k] = v
                arr = newarr
            self.update(arr)
        else:
            raise TypeError("dict expected at most 1 argument, got 3")

    def update(self, arr, **_f):
        # note, a ValueError halfway through copying values
        # will leave half the values in this dict.
        for k, v in arr.items():
            self.__setitem__(k, v)

    def raw_get(self, key):
        return dict.__getitem__(self, key)

    def __setitem__(self, key, value):
        if not isinstance(key, PdfObject):
            raise ValueError("key must be PdfObject")
        if not isinstance(value, PdfObject):
            raise ValueError("value must be PdfObject")
        return dict.__setitem__(self, key, value)

    def setdefault(self, key, value=None):
        if not isinstance(key, PdfObject):
            raise ValueError("key must be PdfObject")
        if not isinstance(value, PdfObject):
            raise ValueError("value must be PdfObject")
        return dict.setdefault(self, key, value)

    def __getitem__(self, key):
        utils.debug(key, dict.__getitem__(self, key))
        return dict.__getitem__(self, key).get_object()

    ##
    # Retrieves XMP (Extensible Metadata Platform) data relevant to the
    # this object, if available.
    # <p>
    # Stability: Added in v1.12, will exist for all future v1.x releases.
    # @return Returns a {@link #xmp.XmpInformation XmlInformation} instance
    # that can be used to access XMP metadata from the document.  Can also
    # return None if no metadata was found on the document root.
    def get_xmp_metadata(self):
        metadata = self.get("/Metadata", None)
        if metadata is None:
            return None
        metadata = metadata.getObject()
        import PyPDF.xmp as xmp
        if not isinstance(metadata, xmp.XmpInformation):
            metadata = xmp.XmpInformation(metadata)
            self[NameObject("/Metadata")] = metadata
        return metadata

    ##
    # Read-only property that accesses the {@link
    # #DictionaryObject.getXmpData getXmpData} function.
    # <p>
    # Stability: Added in v1.12, will exist for all future v1.x releases.
    xmpMetadata = property(lambda self: self.get_xmp_metadata(), None, None)

    def write_to_stream(self, stream, encryption_key):
        stream.write(b'<<\n')
        for key, value in self.items():
            key.write_to_stream(stream, encryption_key)
            stream.write(b' ')
            value.write_to_stream(stream, encryption_key)
            stream.write(b'\n')
        stream.write(b'>>')

    @staticmethod
    def read_from_stream(stream, pdf_object):
        tmp = stream.read(2)
        if not tmp == b'<<':
            raise utils.PdfReadError("dictionary read error")
        data = {}
        while True:
            tok = read_non_whitespace(stream)
            if tok in b'>':
                stream.read(1)
                break
            stream.seek(-1, io.SEEK_CUR)
            key = read_object(stream, pdf_object)
            seek_token(stream)
            value = read_object(stream, pdf_object)
            if key in data:
                # multiple definitions of key not permitted
                raise utils.PdfReadError("multiple definitions in dictionary")
            data[key] = value
        pos = stream.tell()
        s = read_non_whitespace(stream)
        if s in b's' and stream.read(5) == b'tream':
            eol = stream.read(1)
            # odd PDF file output has spaces after 'stream' keyword but before EOL.
            # patch provided by Danial Sandler
            while eol == b' ':
                eol = stream.read(1)
            assert eol in b'\n\r'
            if eol in b'\r':
                # read \n after
                stream.read(1)
            # this is a stream object, not a dictionary
            assert "/Length" in data
            length = data["/Length"]
            if isinstance(length, IndirectObject):
                t = stream.tell()
                length = pdf_object.get_object(length)
                stream.seek(t, io.SEEK_SET)
            data["__streamdata__"] = stream.read(length)
            e = read_non_whitespace(stream)
            ndstream = stream.read(8)
            if (e + ndstream) != "endstream":
                # (sigh) - the odd PDF file has a length that is too long, so
                # we need to read backwards to find the "endstream" ending.
                # ReportLab (unknown version) generates files with this bug,
                # and Python users into PDF files tend to be our audience.
                # we need to do this to correct the streamdata and chop off
                # an extra character.
                pos = stream.tell()
                stream.seek(-10, io.SEEK_CUR)
                end = stream.read(9)
                if end == "endstream":
                    # we found it by looking back one character further.
                    data["__streamdata__"] = data["__streamdata__"][:-1]
                else:
                    stream.seek(pos, io.SEEK_SET)
                    raise utils.PdfReadError("Unable to find 'endstream' marker after stream.")
        else:
            stream.seek(pos, io.SEEK_SET)
        if "__streamdata__" in data:
            return StreamObject.initialize_from_dictionary(data)
        else:
            retval = DictionaryObject()
            retval.update(data)
            return retval


class StreamObject(DictionaryObject):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._data = None
        self.decodedSelf = None

    def write_to_stream(self, stream, encryption_key):
        self[NameObject("/Length")] = NumberObject(len(self._data))
        DictionaryObject.write_to_stream(self, stream, encryption_key)
        del self["/Length"]
        stream.write("\nstream\n")
        data = self._data
        if encryption_key:
            data = rc4_encrypt(encryption_key, data)
        stream.write(data)
        stream.write("\nendstream")

    @staticmethod
    def initialize_from_dictionary(data):
        if "/Filter" in data:
            retval = EncodedStreamObject()
        else:
            retval = DecodedStreamObject()
        retval._data = data["__streamdata__"]
        del data["__streamdata__"]
        del data["/Length"]
        retval.update(data)
        return retval

    def flate_encode(self):
        if "/Filter" in self:
            f = self["/Filter"]
            if isinstance(f, ArrayObject):
                f.insert(0, NameObject("/FlateDecode"))
            else:
                newf = ArrayObject()
                newf.append(NameObject("/FlateDecode"))
                newf.append(f)
                f = newf
        else:
            f = NameObject("/FlateDecode")
        retval = EncodedStreamObject()
        retval[NameObject("/Filter")] = f
        retval._data = filters.FlateDecode.encode(self._data)
        return retval

    def get_data(self):
        return self._data

    def set_data(self, data):
        self._data = data


class DecodedStreamObject(StreamObject):
    pass


class EncodedStreamObject(StreamObject):
    def __init__(self):
        super().__init__()
        self.decodedSelf = None

    def get_data(self):
        if self.decodedSelf:
            # cached version of decoded object
            return self.decodedSelf.get_data()
        else:
            # create decoded object
            decoded = DecodedStreamObject()
            decoded._data = filters.decode_stream_data(self)
            for key, value in self.items():
                if key not in ("/Length", "/Filter", "/DecodeParms"):
                    decoded[key] = value
            self.decodedSelf = decoded
            return decoded.get_data()

    def set_data(self, data):
        raise utils.PdfReadError("Creating EncodedStreamObject is not currently supported")


class RectangleObject(ArrayObject):
    def __init__(self, arr):
        # must have four points
        assert len(arr) == 4
        # automatically convert arr[x] into NumberObject(arr[x]) if necessary
        ArrayObject.__init__(self, [self.ensure_is_number(x) for x in arr])

    @staticmethod
    def ensure_is_number(value):
        if not isinstance(value, (NumberObject, FloatObject)):
            value = FloatObject(value)
        return value

    def __repr__(self):
        return "RectangleObject(%s)" % repr(list(self))

    def get_lower_left_x(self):
        return self[0]

    def get_lower_left_y(self):
        return self[1]

    def get_upper_right_x(self):
        return self[2]

    def get_upper_right_y(self):
        return self[3]

    def get_upper_left_x(self):
        return self.get_lower_left_x()

    def get_upper_left_y(self):
        return self.get_upper_right_y()

    def get_lower_right_x(self):
        return self.get_upper_right_x()

    def get_lower_right_y(self):
        return self.get_lower_left_y()

    def get_lower_left(self):
        return self.get_lower_left_x(), self.get_lower_left_y()

    def get_lower_right(self):
        return self.get_lower_right_x(), self.get_lower_right_y()

    def get_upper_left(self):
        return self.get_upper_left_x(), self.get_upper_left_y()

    def get_upper_right(self):
        return self.get_upper_right_x(), self.get_upper_right_y()

    def set_lower_left(self, value):
        self[0], self[1] = [self.ensure_is_number(x) for x in value]

    def set_lower_right(self, value):
        self[2], self[1] = [self.ensure_is_number(x) for x in value]

    def set_upper_left(self, value):
        self[0], self[3] = [self.ensure_is_number(x) for x in value]

    def set_upper_right(self, value):
        self[2], self[3] = [self.ensure_is_number(x) for x in value]

    def get_width(self):
        return self.get_upper_right_x() - self.get_lower_left_x()

    def get_height(self):
        return self.get_upper_right_y() - self.get_lower_left_x()


##
# Given a string (either a "str" or "unicode"), create a ByteStringObject or a
# TextStringObject to represent the string.
def create_string_object(string: bytes):
    if isinstance(string, bytes):
        if string.startswith(codecs.BOM_UTF16_BE):
            retval = TextStringObject(string.decode(utils.ENCODING_UTF16))
            retval.autodetect_utf16 = True
            return retval
        else:
            # This is probably a big performance hit here, but we need to
            # convert string objects into the text/unicode-aware version if
            # possible... and the only way to check if that's possible is
            # to try.  Some strings are strings, some are just byte arrays.
            try:
                retval = TextStringObject(decode_pdfdocencoding(string))
                retval.autodetect_pdfdocencoding = True
                return retval
            except UnicodeDecodeError:
                return ByteStringObject(string)
    elif isinstance(string, str):
        return TextStringObject(string)
    else:
        raise TypeError("createStringObject should have str or unicode arg")


def read_hex_string_from_stream(stream):
    stream.read(1)
    txt = b''
    x = b''
    while True:
        tok = read_non_whitespace(stream)
        if tok == b'>':
            break
        x += tok
        if len(x) == 2:
            txt += utils.s2b(chr(int(x, base=16)))
            x = b''
    if len(x) == 1:
        x += b'0'
    if len(x) == 2:
        txt += utils.s2b(chr(int(x, base=16)))
    return create_string_object(txt)


def read_string_from_stream(stream):
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
                tok = utils.s2b(chr(int(tok, base=8)))
            elif tok in b'\n\r':
                # This case is  hit when a backslash followed by a line
                # break occurs.  If it's a multi-char EOL, consume the
                # second character:
                tok = stream.read(1)
                if tok not in b'\n\r':
                    stream.seek(-1, io.SEEK_CUR)
                # Then don't add anything to the actual string, since this
                # line break was escaped:
                tok = b''
            else:
                raise utils.PdfReadError("Unexpected escaped string")
        txt += tok
    return create_string_object(txt)


def encode_pdfdocencoding(unicode_string):
    retval = ''
    for c in unicode_string:
        try:
            retval += chr(_pdfDocEncoding_rev[c])
        except KeyError:
            raise UnicodeEncodeError("pdfdocencoding", c, -1, -1, "does not exist in translation table")
    return retval


def decode_pdfdocencoding(byte_array: bytes):
    retval = u''
    for b in byte_array:
        c = _pdfDocEncoding[b]
        if c == u'\u0000':
            raise UnicodeDecodeError("pdfdocencoding", bytes([b]), -1, -1, "does not exist in translation table")
        retval += c
    utils.debug(retval)
    return retval


_pdfDocEncoding = (
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

assert len(_pdfDocEncoding) == 256

_pdfDocEncoding_rev = {}
for __idx in range(256):
    char = _pdfDocEncoding[__idx]
    if char == u"\u0000":
        continue
    assert char not in _pdfDocEncoding_rev
    _pdfDocEncoding_rev[char] = __idx
