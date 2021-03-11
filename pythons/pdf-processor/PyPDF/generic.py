# coding: utf-8
"""
Implementation of generic PDF objects (dictionary, number, string, and so on)
"""

import io
import re
import PyPDF.filters as _f
import PyPDF.utils as _u
import PyPDF.keys as _k
import decimal
import codecs

_STREAM_KEY = "__streamdata__"


def read_object(stream, pdf_reader):
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
        _u.seek_token(stream)
        return read_object(stream, pdf_reader)
    else:
        # number object OR indirect reference
        if tok in b'+-':
            # number
            return NumberObject.read_from_stream(stream)
        peek = stream.read(20)
        stream.seek(-len(peek), io.SEEK_CUR)  # reset to start
        if re.match(br'(\d+)\s(\d+)\sR[^a-zA-Z]', peek) is not None:
            return Reference.read_from_stream(stream, pdf_reader)
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
            raise _u.PdfReadError("error reading null object")
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
            if is_plain_object(data):
                data.write_to_stream(stream)
            else:
                data.write_to_stream(stream, encryption_key)
        stream.write(b' ]')

    @staticmethod
    def read_from_stream(stream, pdf):
        arr = ArrayObject()
        tmp = stream.read(1)
        if tmp not in b'[':
            raise _u.PdfReadError("error reading array")
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


class Reference(PdfObject):
    def __init__(self, idnum, generation, pdf):
        self.idnum = idnum
        # XXX Seems that generation is always 0.
        self.generation = generation
        self.parent = pdf

    def get_object(self):
        return self.parent.get_object(self).get_object()

    def __repr__(self):
        return 'Reference(%r, %r, %s)' % (self.idnum, self.generation, type(self.get_object()).__name__)

    def __eq__(self, other):
        return (other is not None and
                isinstance(other, Reference) and
                self.idnum == other.idnum and
                self.generation == other.generation and
                self.parent is other.parent)

    def __ne__(self, other):
        return not self.__eq__(other)

    def write_to_stream(self, stream):
        stream.write(bytes('%s %s R' % (self.idnum, self.generation), _u.ENCODING_UTF8))

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
            raise _u.PdfReadError("error reading indirect object reference")
        return Reference(int(idnum), int(generation), pdf)


class FloatObject(decimal.Decimal, PdfObject):
    def __new__(cls, value='0', context=None):
        # FIXME Warning here.
        # noinspection PyTypeChecker
        return decimal.Decimal.__new__(cls, value, context)

    def __repr__(self):
        if self == self.to_integral():
            return str(self.quantize(decimal.Decimal(1)))
        else:
            return "%.5f" % self  # This adds useless extraneous zeros.

    def write_to_stream(self, stream):
        stream.write(_u.s2b(repr(self)))


class NumberObject(int, PdfObject):
    def __init__(self, value):
        int.__init__(value)

    def write_to_stream(self, stream):
        stream.write(bytes(repr(self), _u.ENCODING_UTF8))

    @staticmethod
    def read_from_stream(stream):
        name = b''
        while True:
            tok = stream.read(1)
            if not (tok in b'+-.' or tok.isdigit()):
                stream.seek(-1, io.SEEK_CUR)
                break
            name += tok
        if name.find(b'.') > -1:
            return FloatObject(name.decode(_u.ENCODING_UTF8))
        else:
            return NumberObject(name)


##
# Represents a string object where the text encoding could not be determined.
# This occurs quite often, as the PDF spec doesn't provide an alternate way to
# represent strings -- for example, the encryption data stored in files (like
# /O) is clearly not text, but is still stored in a "String" object.
class ByteStringObject(str, PdfObject):
    @property
    def original_bytes(self):
        return self

    def write_to_stream(self, stream, encryption_key):
        bytearr = self
        if encryption_key:
            bytearr = _u.rc4_encrypt(encryption_key, bytearr)
        stream.write(b'<')
        stream.write(bytearr.encode("hex"))
        stream.write(b'>')


##
# Represents a string object that has been decoded into a real unicode string.
# If read from a PDF document, this string appeared to match the
# PDFDocEncoding, or contained a UTF-16BE BOM mark to cause UTF-16 decoding to
# occur.
class TextStringObject(str, PdfObject):
    autodetect_pdfdocencoding = False
    autodetect_utf16 = False

    @property
    def original_bytes(self):
        """
        It is occasionally possible that a text string object gets created where
        a byte string object was expected due to the autodetection mechanism --
        if that occurs, this "original_bytes" property can be used to
        back-calculate what the original encoded bytes were.
        """
        # We're a text string object, but the library is trying to get our raw
        # bytes.  This can happen if we auto-detected this string as text, but
        # we were wrong.  It's pretty common.  Return the original bytes that
        # would have been used to create this object, based upon the autodetect
        # method.
        if self.autodetect_utf16:
            return codecs.BOM_UTF16_BE + self.encode(_u.ENCODING_UTF16BE)
        elif self.autodetect_pdfdocencoding:
            return _u.encode_pdf_doc_encoding(self)
        else:
            raise Exception("no information about original bytes")

    def write_to_stream(self, stream, encryption_key):
        # Try to write the string out as a PDFDocEncoding encoded string.  It's
        # nicer to look at in the PDF file.  Sadly, we take a performance hit
        # here for trying...
        try:
            bytearr = _u.encode_pdf_doc_encoding(self)
        except UnicodeEncodeError:
            bytearr = codecs.BOM_UTF16_BE + self.encode(_u.ENCODING_UTF16BE)
        if encryption_key:
            bytearr = _u.rc4_encrypt(encryption_key, bytearr)
            obj = ByteStringObject(bytearr)
            obj.write_to_stream(stream, None)
        else:
            stream.write(b'(')
            for c in bytearr:
                if not c.isalnum() and not c == ' ':
                    stream.write(_u.s2b("\\%03o" % ord(c)))
                else:
                    stream.write(_u.s2b(c))
            stream.write(b')')


class NameObject(bytes, PdfObject):
    def __init__(self, data):
        bytes.__init__(data)

    def write_to_stream(self, stream):
        stream.write(self)

    @staticmethod
    def read_from_stream(stream):
        name = stream.read(1)
        if name not in b'/':
            raise _u.PdfReadError("name read error")
        while True:
            tok = stream.read(1)
            if tok.isspace() or tok in _u.DELIMITERS:
                stream.seek(-1, io.SEEK_CUR)
                break
            name += tok
        return NameObject(name)


def read_stream_object_with(data, pdf_object, stream):
    pos = stream.tell()
    s = _u.read_non_whitespace(stream)
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
        assert b'/Length' in data
        length = data[b'/Length']
        if isinstance(length, Reference):
            t = stream.tell()
            length = pdf_object.get_object(length)
            stream.seek(t, io.SEEK_SET)
        data[_STREAM_KEY] = stream.read(length)
        e = _u.read_non_whitespace(stream)
        ndstream = stream.read(8)
        if (e + ndstream) != b'endstream':
            # (sigh) - the odd PDF file has a length that is too long, so
            # we need to read backwards to find the "endstream" ending.
            # ReportLab (unknown version) generates files with this bug,
            # and Python users into PDF files tend to be our audience.
            # we need to do this to correct the streamdata and chop off
            # an extra character.
            pos = stream.tell()
            stream.seek(-10, io.SEEK_CUR)
            end = stream.read(9)
            if end == b'endstream':
                # we found it by looking back one character further.
                data[_STREAM_KEY] = data[_STREAM_KEY][:-1]
            else:
                stream.seek(pos, io.SEEK_SET)
                raise _u.PdfReadError("Unable to find 'endstream' marker after stream.")
    else:
        stream.seek(pos, io.SEEK_SET)


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

    def __getitem__(self, key):
        return dict.__getitem__(self, key).get_object()

    def setdefault(self, key, value=None):
        if not isinstance(key, PdfObject):
            raise ValueError("key must be PdfObject")
        if not isinstance(value, PdfObject):
            raise ValueError("value must be PdfObject")
        return dict.setdefault(self, key, value)

    def get_xmp_metadata(self):
        """
        Retrieves XMP (Extensible Metadata Platform) data relevant to the
        this object, if available.

        Stability: Added in v1.12, will exist for all future v1.x releases.
        @return Returns a {@link #xmp.XmpInformation XmlInformation} instance
        that can be used to access XMP metadata from the document.  Can also
        return None if no metadata was found on the document root.
        """
        metadata = self.get(b'/Metadata', None)
        if metadata is None:
            return None
        metadata = metadata.getObject()
        import PyPDF.xmp as xmp
        if not isinstance(metadata, xmp.XmpInformation):
            metadata = xmp.XmpInformation(metadata)
            self[NameObject(b'/Metadata')] = metadata
        return metadata

    def write_to_stream(self, stream, encryption_key=None):
        stream.write(b'<<\n')
        for key, value in self.items():
            key.write_to_stream(stream)
            stream.write(b' ')
            if is_plain_object(value):
                value.write_to_stream(stream)
            else:
                value.write_to_stream(stream, encryption_key)
            stream.write(b'\n')
        stream.write(b'>>')

    @staticmethod
    def read_from_stream(stream, pdf_object):
        tmp = stream.read(2)
        if not tmp == b'<<':
            raise _u.PdfReadError("dictionary read error")
        data = {}
        while True:
            tok = _u.read_non_whitespace(stream)
            if tok in b'>':
                stream.read(1)
                break
            stream.seek(-1, io.SEEK_CUR)
            key = read_object(stream, pdf_object)
            _u.seek_token(stream)
            value = read_object(stream, pdf_object)
            if key in data:
                # multiple definitions of key not permitted
                raise _u.PdfReadError("multiple definitions in dictionary")
            data[key] = value
        read_stream_object_with(data, pdf_object, stream)
        if _STREAM_KEY in data:
            return initialize_from_dictionary(data)
        else:
            retval = DictionaryObject()
            retval.update(data)
            return retval


class StreamObject(DictionaryObject):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._bytes_data = None
        self._decoded_stream = None

    def write_to_stream(self, stream, encryption_key=None):
        self[NameObject(b'/Length')] = NumberObject(len(self._bytes_data))
        DictionaryObject.write_to_stream(self, stream, encryption_key)
        del self[b'/Length']
        stream.write(b'\nstream\n')
        data = self._bytes_data
        if encryption_key is not None:
            data = _u.rc4_encrypt(encryption_key, data)
        stream.write(data)
        stream.write(b'\nendstream')

    def flate_encode(self):
        if b'/Filter' in self:
            f = self[b'/Filter']
            if isinstance(f, ArrayObject):
                f.insert(0, NameObject(_k.FLATE_DECODE))
            else:
                newf = ArrayObject()
                newf.append(NameObject(_k.FLATE_DECODE))
                newf.append(f)
                f = newf
        else:
            f = NameObject(_k.FLATE_DECODE)
        retval = _EncodedStreamObject()
        retval[NameObject(b'/Filter')] = f
        retval._bytes_data = _f.FlateDecode.encode(self._bytes_data)
        return retval

    def get_data(self):
        return self._bytes_data

    def set_data(self, data):
        self._bytes_data = data


def initialize_from_dictionary(data):
    if b'/Filter' in data:
        retval = _EncodedStreamObject()
    else:
        retval = DecodedStreamObject()
    retval._bytes_data = data[_STREAM_KEY]
    del data[_STREAM_KEY]
    del data[b'/Length']
    retval.update(data)
    return retval


class DecodedStreamObject(StreamObject):
    pass


class _EncodedStreamObject(StreamObject):
    def __init__(self):
        super().__init__()
        self._decoded_stream = None

    @property
    def bytes_data(self):
        return self._bytes_data

    def get_data(self):
        if self._decoded_stream is not None:
            # cached version of decoded object
            return self._decoded_stream.get_data()
        else:
            # assert self.decodedSelf is not None
            # create decoded object
            decoded = DecodedStreamObject()
            decoded._bytes_data = _decode_stream_data(self)
            for key, value in self.items():
                if key not in (b'/Length', b'/Filter', b'/DecodeParms'):
                    decoded[key] = value
            self._decoded_stream = decoded
            return decoded.get_data()

    def set_data(self, data):
        raise _u.PdfReadError("Creating EncodedStreamObject is not currently supported")


class DocumentInformation(DictionaryObject):
    """
    A class representing the basic document metadata provided in a PDF File.

    As of pyPdf v1.10, all text properties of the document metadata have two
    properties, eg. author and author_raw.  The non-raw property will always
    return a TextStringObject, making it ideal for a case where the metadata is
    being displayed.  The raw property can sometimes return a ByteStringObject,
    if pyPdf was unable to decode the string's text encoding; this requires
    additional safety in the caller and therefore is not as commonly accessed.
    """

    def __init__(self):
        DictionaryObject.__init__(self)

    def get_text(self, key):
        # XXX Keys: b'/Producer', b'/Title', b'/Author', b'/Subject', b'/Creator',
        retval = self.get(key, None)
        if isinstance(retval, TextStringObject):
            return retval
        return None


class Destination(DictionaryObject):
    """
    A class representing a destination within a PDF file.
    See section 8.2.1 of the PDF 1.6 reference.
    Stability: Added in v1.10, will exist for all v1.x releases.
    """

    def __init__(self, title, page, position_type, *args):
        DictionaryObject.__init__(self)
        self[NameObject(b'/Title')] = title
        self[NameObject(_k.PAGE)] = page
        self[NameObject(_k.TYPE)] = position_type

        # from table 8.2 of the PDF 1.6 reference.
        if position_type == b'/XYZ':
            (self[NameObject(b'/Left')], self[NameObject(b'/Top')],
             self[NameObject(b'/Zoom')]) = args
        elif position_type == b'/FitR':
            (self[NameObject(b'/Left')], self[NameObject(b'/Bottom')],
             self[NameObject(b'/Right')], self[NameObject(b'/Top')]) = args
        elif position_type in [b'/FitH', b'FitBH']:
            self[NameObject(b'/Top')], = args
        elif position_type in [b'/FitV', b'FitBV']:
            self[NameObject(b'/Left')], = args
        elif position_type in [b'/Fit', b'FitB']:
            pass
        else:
            raise _u.PdfReadError("Unknown Destination Type: %r" % position_type)


def create_string_object(string):
    if isinstance(string, bytes):
        if string.startswith(codecs.BOM_UTF16_BE):
            retval = TextStringObject(string.decode(_u.ENCODING_UTF16))
            retval.autodetect_utf16 = True
            return retval
        else:
            # This is probably a big performance hit here, but we need to
            # convert string objects into the text/unicode-aware version if
            # possible... and the only way to check if that's possible is
            # to try.  Some strings are strings, some are just byte arrays.
            try:
                retval = TextStringObject(_u.decode_pdf_doc_encoding(string))
                retval.autodetect_pdfdocencoding = True
                return retval
            except UnicodeDecodeError:
                return ByteStringObject(string)
    elif isinstance(string, str):
        return TextStringObject(string)
    else:
        raise TypeError("createStringObject should have str or unicode arg")


def read_hex_string_from_stream(stream):
    return create_string_object(_u.read_hex_bytes_from(stream))


def read_string_from_stream(stream):
    return create_string_object(_u.read_bytes_from(stream))


def is_plain_object(obj):
    return isinstance(obj, _PLAIN_OBJECTS)


def _decode_stream_data(stream):
    filters_in_steam = stream.get(b'/Filter', ())
    if len(filters_in_steam) and not isinstance(filters_in_steam[0], NameObject):
        # we have a single filter instance
        filters_in_steam = (filters_in_steam,)
    data = stream.bytes_data
    for filterType in filters_in_steam:
        if filterType == _k.FLATE_DECODE:
            data = _f.FlateDecode.decode(data, stream.get(b'/DecodeParms'))
        elif filterType == b'/ASCIIHexDecode':
            data = _f.ASCIIHexDecode.decode(data)
        elif filterType == b'/ASCII85Decode':
            data = _f.ASCII85Decode.decode(data)
        elif filterType == b'/DCTDecode':
            data = _f.DCTDecode.decode(data)
        elif filterType == b'/JPXDecode':
            data = _f.JPXDecode.decode(data)
        elif filterType == b'/CCITTFaxDecode':
            height = stream.get(b'/Height', ())
            data = _f.CCITTFaxDecode.decode(data, stream.get(b'/DecodeParms'), height)
        elif filterType == b'/Crypt':
            decode_params = stream.get(b'/DecodeParams', {})
            if b'/Name' not in decode_params and _k.TYPE not in decode_params:
                pass
            else:
                raise NotImplementedError("/Crypt filter with /Name or /Type not supported yet")
        else:
            raise NotImplementedError("unsupported filter %s" % filterType)
    return data


_PLAIN_OBJECTS = (Reference, NumberObject, NameObject, FloatObject, BooleanObject)
