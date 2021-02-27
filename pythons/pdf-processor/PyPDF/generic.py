# coding: utf-8
"""
Implementation of generic PDF objects (dictionary, number, string, and so on)
"""

import io
import re
import math
from io import BytesIO
from PyPDF.utils import read_non_whitespace, seek_token, rc4_encrypt
import PyPDF.filters as filters
import PyPDF.utils as utils
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
        return 'IndirectObject(%r, %r, %s)' % (self.idnum, self.generation, type(self.get_object()))

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
            return str(self.quantize(decimal.Decimal(1)))
        else:
            return "%.5f" % self  # This adds useless extraneous zeros.

    def write_to_stream(self, stream):
        stream.write(utils.s2b(repr(self)))


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
    @property
    def original_bytes(self):
        return self

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
            return codecs.BOM_UTF16_BE + self.encode(utils.ENCODING_UTF16BE)
        elif self.autodetect_pdfdocencoding:
            return encode_pdf_doc_encoding(self)
        else:
            raise Exception("no information about original bytes")

    def write_to_stream(self, stream, encryption_key):
        # Try to write the string out as a PDFDocEncoding encoded string.  It's
        # nicer to look at in the PDF file.  Sadly, we take a performance hit
        # here for trying...
        try:
            bytearr = encode_pdf_doc_encoding(self)
        except UnicodeEncodeError:
            bytearr = codecs.BOM_UTF16_BE + self.encode(utils.ENCODING_UTF16BE)
        if encryption_key:
            bytearr = rc4_encrypt(encryption_key, bytearr)
            obj = ByteStringObject(bytearr)
            obj.write_to_stream(stream, None)
        else:
            stream.write(b'(')
            for c in bytearr:
                if not c.isalnum() and not c == ' ':
                    stream.write(utils.s2b("\\%03o" % ord(c)))
                else:
                    stream.write(utils.s2b(c))
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
        metadata = self.get(b'/Metadata', None)
        if metadata is None:
            return None
        metadata = metadata.getObject()
        import PyPDF.xmp as xmp
        if not isinstance(metadata, xmp.XmpInformation):
            metadata = xmp.XmpInformation(metadata)
            self[NameObject(b'/Metadata')] = metadata
        return metadata

    def write_to_stream(self, stream, encryption_key):
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
            assert b'/Length' in data
            length = data[b'/Length']
            if isinstance(length, IndirectObject):
                t = stream.tell()
                length = pdf_object.get_object(length)
                stream.seek(t, io.SEEK_SET)
            data[_STREAM_KEY] = stream.read(length)
            e = read_non_whitespace(stream)
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
                    raise utils.PdfReadError("Unable to find 'endstream' marker after stream.")
        else:
            stream.seek(pos, io.SEEK_SET)
        if _STREAM_KEY in data:
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
        self[NameObject(b'/Length')] = NumberObject(len(self._data))
        DictionaryObject.write_to_stream(self, stream, encryption_key)
        del self[b'/Length']
        stream.write(b'\nstream\n')
        data = self._data
        if encryption_key:
            data = rc4_encrypt(encryption_key, data)
        stream.write(data)
        stream.write(b'\nendstream')

    @staticmethod
    def initialize_from_dictionary(data):
        if b'/Filter' in data:
            retval = EncodedStreamObject()
        else:
            retval = DecodedStreamObject()
        retval._data = data[_STREAM_KEY]
        del data[_STREAM_KEY]
        del data[b'/Length']
        retval.update(data)
        return retval

    def flate_encode(self):
        if b'/Filter' in self:
            f = self[b'/Filter']
            if isinstance(f, ArrayObject):
                f.insert(0, NameObject(b'/FlateDecode'))
            else:
                newf = ArrayObject()
                newf.append(NameObject(b'/FlateDecode'))
                newf.append(f)
                f = newf
        else:
            f = NameObject(b'/FlateDecode')
        retval = EncodedStreamObject()
        retval[NameObject(b'/Filter')] = f
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

    @property
    def bytes_data(self):
        return self._data

    def get_data(self):
        if self.decodedSelf is not None:
            # cached version of decoded object
            return self.decodedSelf.get_data()
        else:
            # assert self.decodedSelf is not None
            # create decoded object
            decoded = DecodedStreamObject()
            decoded._data = filters.decode_stream_data(self)
            for key, value in self.items():
                if key not in (b'/Length', b'/Filter', b'/DecodeParms'):
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


def _create_rectangle_accessor(name, fallback):
    return property(
        lambda self: _get_rectangle(self, name, fallback),
        lambda self, value: _set_rectangle(self, name, value),
        lambda self: _delete_rectangle(self, name)
    )


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
        self[NameObject(b'/Page')] = page
        self[NameObject(b'/Type')] = position_type

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
            raise utils.PdfReadError("Unknown Destination Type: %r" % position_type)


class PageObject(DictionaryObject):
    """
    This class represents a single page within a PDF file.  Typically this object
    will be created by accessing the PdfFileReader.get_page function of the
    PdfFileReader class, but it is also possible to create an empty page
    with the create_blank_page static method.

    Constructor arguments:
    _pdf -- PDF file the page belongs to (optional, defaults to None).
    """

    def __init__(self, _pdf=None, indirect_ref=None):
        DictionaryObject.__init__(self)
        self.pdf = _pdf
        # Stores the original indirect reference to this object in its source PDF
        self.indirect_ref = indirect_ref

    def create_blank_page(_pdf=None, width=None, height=None):
        """
        Returns a new blank page.
        If width or height is None, try to get the page size from the
        last page of PyPDF. If PyPDF is None or contains no page, a
        PageSizeNotDefinedError is raised.

         _pdf -- PDF file the page belongs to
        width -- The width of the new page expressed in default user space units.
        height -- The height of the new page expressed in default user space units.
        """
        page = PageObject(_pdf)

        # Creates a new page (cf PDF Reference  7.7.3.3)
        page.__setitem__(NameObject(b'/Type'), NameObject(b'/Page'))
        page.__setitem__(NameObject(b'/Parent'), NullObject())
        page.__setitem__(NameObject(b'/Resources'), DictionaryObject())
        if width is None or height is None:
            if _pdf is not None and _pdf.get_pages_count() > 0:
                lastpage = _pdf.get_page(_pdf.get_pages_count() - 1)
                width = lastpage.media_box.get_width()
                height = lastpage.media_box.get_height()
            else:
                raise utils.PageSizeNotDefinedError()
        page.__setitem__(NameObject(b'/MediaBox'), RectangleObject([0, 0, width, height]))
        return page

    create_blank_page = staticmethod(create_blank_page)

    ##
    # Rotates a page clockwise by increments of 90 degrees.
    # <p>
    # Stability: Added in v1.1, will exist for all future v1.x releases.
    # @param angle Angle to rotate the page.  Must be an increment of 90 deg.
    def rotate_clockwise(self, angle):
        assert angle % 90 == 0
        self._rotate(angle)
        return self

    ##
    # Rotates a page counter-clockwise by increments of 90 degrees.
    # <p>
    # Stability: Added in v1.1, will exist for all future v1.x releases.
    # @param angle Angle to rotate the page.  Must be an increment of 90 deg.
    def rotate_counter_clockwise(self, angle):
        assert angle % 90 == 0
        self._rotate(-angle)
        return self

    def _rotate(self, angle):
        current_angle = self.get(b'/Rotate', 0)
        self[NameObject(b'/Rotate')] = NumberObject(current_angle + angle)

    def _merge_resources(res1, res2, resource):
        new_res = DictionaryObject()
        new_res.update(res1.get(resource, DictionaryObject()).get_object())
        page2_res = res2.get(resource, DictionaryObject()).get_object()
        rename_res = {}
        for key in page2_res.keys():
            if key in new_res and new_res[key] != page2_res[key]:
                newname = NameObject(key + b'renamed')
                rename_res[key] = newname
                new_res[newname] = page2_res[key]
            elif key not in new_res:
                new_res[key] = page2_res.raw_get(key)
        return new_res, rename_res

    _merge_resources = staticmethod(_merge_resources)

    def _content_stream_rename(stream, rename, _pdf):
        if not rename:
            return stream
        stream = ContentStream(stream, _pdf)
        for operands, operator in stream.operations:
            for i in range(len(operands)):
                op = operands[i]
                if isinstance(op, NameObject):
                    operands[i] = rename.get(op, op)
        return stream

    _content_stream_rename = staticmethod(_content_stream_rename)

    def _push_pop_gs(contents, _pdf):
        # adds a graphics state "push" and "pop" to the beginning and end
        # of a content stream.  This isolates it from changes such as
        # transformation matricies.
        stream = ContentStream(contents, _pdf)
        stream.operations.insert(0, [[], b'q'])
        stream.operations.append([[], b'Q'])
        return stream

    _push_pop_gs = staticmethod(_push_pop_gs)

    def _add_transformation_matrix(contents, _pdf, ctm):
        # adds transformation matrix at the beginning of the given
        # contents stream.
        a, b, c, d, e, f = ctm
        contents = ContentStream(contents, _pdf)
        contents.operations.insert(0, [[FloatObject(a), FloatObject(b),
                                        FloatObject(c), FloatObject(d), FloatObject(e),
                                        FloatObject(f)], " cm"])
        return contents

    _add_transformation_matrix = staticmethod(_add_transformation_matrix)

    ##
    # Returns the /Contents object, or None if it doesn't exist.
    # /Contents is optionnal, as described in PDF Reference  7.7.3.3
    def get_contents(self):
        if b'/Contents' in self:
            return self[b'/Contents'].get_object()
        else:
            return None

    ##
    # Merges the content streams of two pages into one.  Resource references
    # (i.e. fonts) are maintained from both pages.  The mediabox/cropbox/etc
    # of this page are not altered.  The parameter page's content stream will
    # be added to the end of this page's content stream, meaning that it will
    # be drawn after, or "on top" of this page.
    # <p>
    # Stability: Added in v1.4, will exist for all future 1.x releases.
    # @param page2 An instance of {@link #PageObject PageObject} to be merged
    #              into this one.
    def merge_page(self, page2):
        self._merge_page(page2)

    ##
    # Actually merges the content streams of two pages into one. Resource
    # references (i.e. fonts) are maintained from both pages. The
    # mediabox/cropbox/etc of this page are not altered. The parameter page's
    # content stream will be added to the end of this page's content stream,
    # meaning that it will be drawn after, or "on top" of this page.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged
    #              into this one.
    # @param page2transformation A fuction which applies a transformation to
    #                            the content stream of page2. Takes: page2
    #                            contents stream. Must return: new contents
    #                            stream. If omitted, the content stream will
    #                            not be modified.
    def _merge_page(self, page2, page2transformation=None):
        # First we work on merging the resource dictionaries.  This allows us
        # to find out what symbols in the content streams we might need to
        # rename.

        new_resources = DictionaryObject()
        rename = {}
        original_resources = self[b'/Resources'].get_object()
        page2_resources = page2[b'/Resources'].get_object()

        for res in b'/ExtGState', b'/Font', b'/XObject', b'/ColorSpace', b'/Pattern', b'/Shading', b'/Properties':
            new, newrename = PageObject._merge_resources(original_resources, page2_resources, res)
            if new:
                new_resources[NameObject(res)] = new
                rename.update(newrename)

        # Combine /ProcSet sets.
        new_resources[NameObject(b'/ProcSet')] = ArrayObject(
            frozenset(original_resources.get(b'/ProcSet', ArrayObject()).get_object()).union(
                frozenset(page2_resources.get(b'/ProcSet', ArrayObject()).get_object())
            )
        )

        new_content_array = ArrayObject()

        original_content = self.get_contents()
        if original_content is not None:
            new_content_array.append(PageObject._push_pop_gs(original_content, self.pdf))

        page2_content = page2.get_contents()
        if page2_content is not None:
            if page2transformation is not None:
                page2_content = page2transformation(page2_content)
            page2_content = PageObject._content_stream_rename(
                page2_content, rename, self.pdf)
            page2_content = PageObject._push_pop_gs(page2_content, self.pdf)
            new_content_array.append(page2_content)

        self[NameObject(b'/Contents')] = ContentStream(new_content_array, self.pdf)
        self[NameObject(b'/Resources')] = new_resources

    ##
    # This is similar to mergePage, but a transformation matrix is
    # applied to the merged stream.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param ctm   A 6 elements tuple containing the operands of the
    #              transformation matrix
    def merge_transformed_page(self, page2, ctm):
        self._merge_page(page2, lambda page2_content: PageObject._add_transformation_matrix(
            page2_content, page2.pdf, ctm
        ))

    ##
    # This is similar to mergePage, but the stream to be merged is scaled
    # by appling a transformation matrix.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param factor The scaling factor
    def merge_scaled_page(self, page2, factor):
        # CTM to scale : [ sx 0 0 sy 0 0 ]
        return self.merge_transformed_page(page2, [factor, 0,
                                                   0, factor,
                                                   0, 0])

    ##
    # This is similar to mergePage, but the stream to be merged is rotated
    # by appling a transformation matrix.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param rotation The angle of the rotation, in degrees
    def merge_rotated_page(self, page2, rotation):
        rotation = math.radians(rotation)
        return self.merge_transformed_page(page2,
                                           [math.cos(rotation), math.sin(rotation),
                                            -math.sin(rotation), math.cos(rotation),
                                            0, 0])

    ##
    # This is similar to mergePage, but the stream to be merged is translated
    # by appling a transformation matrix.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param tx    The translation on X axis
    # @param tx    The translation on Y axis
    def merge_translated_page(self, page2, tx, ty):
        return self.merge_transformed_page(page2, [1, 0,
                                                   0, 1,
                                                   tx, ty])

    ##
    # This is similar to mergePage, but the stream to be merged is rotated
    # and scaled by appling a transformation matrix.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param rotation The angle of the rotation, in degrees
    # @param factor The scaling factor
    def merge_rotated_scaled_page(self, page2, rotation, scale):
        ctm = _create_rotation_scaling_matrix(rotation, scale)

        return self.merge_transformed_page(page2,
                                           [ctm[0][0], ctm[0][1],
                                            ctm[1][0], ctm[1][1],
                                            ctm[2][0], ctm[2][1]])

    ##
    # This is similar to mergePage, but the stream to be merged is translated
    # and scaled by appling a transformation matrix.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param scale The scaling factor
    # @param tx    The translation on X axis
    # @param tx    The translation on Y axis
    def merge_scaled_translated_page(self, page2, scale, tx, ty):
        scaling = [[scale, 0, 0],
                   [0, scale, 0],
                   [0, 0, 1]]
        return self._translate_and_merge(scaling, tx, ty, page2)

    ##
    # This is similar to mergePage, but the stream to be merged is translated,
    # rotated and scaled by appling a transformation matrix.
    #
    # @param page2 An instance of {@link #PageObject PageObject} to be merged.
    # @param tx    The translation on X axis
    # @param ty    The translation on Y axis
    # @param rotation The angle of the rotation, in degrees
    # @param scale The scaling factor
    def merge_rotated_scaled_translated_page(self, page2, rotation, scale, tx, ty):
        ctm = _create_rotation_scaling_matrix(rotation, scale)
        return self._translate_and_merge(ctm, tx, ty, page2)

    def _translate_and_merge(self, ctm, tx, ty, page2):
        translation = [[1, 0, 0],
                       [0, 1, 0],
                       [tx, ty, 1]]
        ctm = utils.matrix_multiply(ctm, translation)

        return self.merge_transformed_page(page2, [ctm[0][0], ctm[0][1],
                                                   ctm[1][0], ctm[1][1],
                                                   ctm[2][0], ctm[2][1]])

    ##
    # Applys a transformation matrix the page.
    #
    # @param ctm   A 6 elements tuple containing the operands of the
    #              transformation matrix
    def add_transformation(self, ctm):
        original_content = self.get_contents()
        if original_content is not None:
            new_content = PageObject._add_transformation_matrix(
                original_content, self.pdf, ctm)
            new_content = PageObject._push_pop_gs(new_content, self.pdf)
            self[NameObject(b'/Contents')] = new_content

    ##
    # Scales a page by the given factors by appling a transformation
    # matrix to its content and updating the page size.
    #
    # @param sx The scaling factor on horizontal axis
    # @param sy The scaling factor on vertical axis
    def scale(self, sx, sy):
        self.add_transformation([sx, 0,
                                 0, sy,
                                 0, 0])
        self.media_box = RectangleObject([
            float(self.media_box.get_lower_left_x()) * sx,
            float(self.media_box.get_lower_left_y()) * sy,
            float(self.media_box.get_upper_right_x()) * sx,
            float(self.media_box.get_upper_right_y()) * sy])

    ##
    # Scales a page by the given factor by appling a transformation
    # matrix to its content and updating the page size.
    #
    # @param factor The scaling factor
    def scale_by(self, factor):
        self.scale(factor, factor)

    ##
    # Scales a page to the specified dimentions by appling a
    # transformation matrix to its content and updating the page size.
    #
    # @param width The new width
    # @param height The new heigth
    def scale_to(self, width, height):
        sx = width / (self.media_box.get_upper_right_x() -
                      self.media_box.get_lower_left_x())
        sy = height / (self.media_box.get_upper_right_y() -
                       self.media_box.get_lower_left_x())
        self.scale(sx, sy)

    ##
    # Compresses the size of this page by joining all content streams and
    # applying a FlateDecode filter.
    # <p>
    # Stability: Added in v1.6, will exist for all future v1.x releases.
    # However, it is possible that this function will perform no action if
    # content stream compression becomes "automatic" for some reason.
    def compress_content_streams(self):
        content = self.get_contents()
        if content is not None:
            if not isinstance(content, ContentStream):
                content = ContentStream(content, self.pdf)
            self[NameObject(b'/Contents')] = content.flate_encode()

    ##
    # Locate all text drawing commands, in the order they are provided in the
    # content stream, and extract the text.  This works well for some PDF
    # files, but poorly for others, depending on the generator used.  This will
    # be refined in the future.  Do not rely on the order of text coming out of
    # this function, as it will change if this function is made more
    # sophisticated.
    # <p>
    # Stability: Added in v1.7, will exist for all future v1.x releases.  May
    # be overhauled to provide more ordered text in the future.
    # @return a unicode string object
    def extract_text(self):
        text = u""
        content = self[b'/Contents'].get_object()
        if not isinstance(content, ContentStream):
            content = ContentStream(content, self.pdf)
        # Note: we check all strings are TextStringObjects.  ByteStringObjects
        # are strings where the byte->string encoding was unknown, so adding
        # them to the text here would be gibberish.
        for operands, operator in content.operations:
            if operator == b'Tj':
                _text = operands[0]
                if isinstance(_text, TextStringObject):
                    text += _text
            elif operator == "T*":
                text += "\n"
            elif operator == "'":
                text += "\n"
                _text = operands[0]
                if isinstance(_text, TextStringObject):
                    text += operands[0]
            elif operator == '"':
                _text = operands[2]
                if isinstance(_text, TextStringObject):
                    text += "\n"
                    text += _text
            elif operator == b'TJ':
                for i in operands[0]:
                    if isinstance(i, TextStringObject):
                        text += i
        return text

    ##
    # A rectangle (RectangleObject), expressed in default user space units,
    # defining the boundaries of the physical medium on which the page is
    # intended to be displayed or printed.
    # <p>
    # Stability: Added in v1.4, will exist for all future v1.x releases.
    media_box = _create_rectangle_accessor(b'/MediaBox', ())

    ##
    # A rectangle (RectangleObject), expressed in default user space units,
    # defining the visible region of default user space.  When the page is
    # displayed or printed, its contents are to be clipped (cropped) to this
    # rectangle and then imposed on the output medium in some
    # implementation-defined manner.  Default value: same as MediaBox.
    # <p>
    # Stability: Added in v1.4, will exist for all future v1.x releases.
    crop_box = _create_rectangle_accessor(b'/CropBox', (b'/MediaBox',))

    ##
    # A rectangle (RectangleObject), expressed in default user space units,
    # defining the region to which the contents of the page should be clipped
    # when output in a production enviroment.
    # <p>
    # Stability: Added in v1.4, will exist for all future v1.x releases.
    bleed_box = _create_rectangle_accessor(b'/BleedBox', (b'/CropBox', b'/MediaBox'))

    ##
    # A rectangle (RectangleObject), expressed in default user space units,
    # defining the intended dimensions of the finished page after trimming.
    # <p>
    # Stability: Added in v1.4, will exist for all future v1.x releases.
    trim_box = _create_rectangle_accessor(b'/TrimBox', (b'/CropBox', b'/MediaBox'))

    ##
    # A rectangle (RectangleObject), expressed in default user space units,
    # defining the extent of the page's meaningful content as intended by the
    # page's creator.
    # <p>
    # Stability: Added in v1.4, will exist for all future v1.x releases.
    art_box = _create_rectangle_accessor(b'/ArtBox', (b'/CropBox', b'/MediaBox'))


class ContentStream(DecodedStreamObject):
    def __init__(self, stream, _pdf, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pdf = _pdf
        self.operations = []
        # stream may be a StreamObject or an ArrayObject containing
        # multiple StreamObjects to be cat'd together.
        stream = stream.get_object()
        if isinstance(stream, ArrayObject):
            data = b''
            for s in stream:
                data += s.get_object().get_data()
            stream = BytesIO(data)
        else:
            stream = BytesIO(stream.get_data())
        # assert stream is None
        self.__parse_content_stream(stream)

    def __parse_content_stream(self, stream):
        stream.seek(0, io.SEEK_SET)
        operands = []
        while True:
            peek = read_non_whitespace(stream)
            if peek == b'':
                break
            stream.seek(-1, io.SEEK_CUR)
            if peek.isalpha() or peek in b'"\'':
                operator = b''
                while True:
                    tok = stream.read(1)
                    if tok == b'':
                        break
                    elif tok.isspace() or tok in utils.DELIMITERS:
                        stream.seek(-1, io.SEEK_CUR)
                        break
                    operator += tok
                if operator == b'BI':
                    # begin inline image - a completely different parsing
                    # mechanism is required, of course... thanks buddy...
                    assert operands == []
                    ii = self._read_inline_image(stream)
                    self.operations.append((ii, b'INLINE IMAGE'))
                else:
                    self.operations.append((operands, operator))
                    operands = []
            elif peek in b'%':
                # If we encounter a comment in the content stream, we have to
                # handle it here.  Typically, readObject will handle
                # encountering a comment -- but readObject assumes that
                # following the comment must be the object we're trying to
                # read.  In this case, it could be an operator instead.
                while peek not in ('\r', '\n'):
                    peek = stream.read(1)
            else:
                operands.append(read_object(stream, None))

    def _read_inline_image(self, stream):
        # begin reading just after the "BI" - begin image
        # first read the dictionary of settings.
        settings = DictionaryObject()
        while True:
            tok = seek_token(stream)
            if tok == b'I':
                # "ID" - begin of image data
                break
            key = read_object(stream, self.pdf)
            seek_token(stream)
            value = read_object(stream, self.pdf)
            settings[key] = value
        # left at beginning of ID
        tmp = stream.read(3)
        assert tmp[:2] == b'ID'
        data = ""
        while True:
            tok = stream.read(1)
            if tok == b'E':
                _next = stream.read(1)
                if _next == b'I':
                    break
                else:
                    stream.seek(-1, io.SEEK_CUR)
                    data += tok
            else:
                data += tok
        seek_token(stream)
        return {b'settings': settings, b'data': data}

    @property
    def _data(self):
        newdata = BytesIO()
        for operands, operator in self.operations:
            if operator == b'INLINE IMAGE':
                newdata.write(b'BI')
                dicttext = BytesIO()
                operands[b'settings'].write_to_stream(dicttext)
                newdata.write(dicttext.getvalue()[2:-2])
                newdata.write(b'ID ')
                newdata.write(operands[b'data'])
                newdata.write(b'EI')
            else:
                for op in operands:
                    op.write_to_stream(newdata)
                    newdata.write(b' ')
                newdata.write(operator)
            newdata.write(b'\n')
        return newdata.getvalue()

    @_data.setter
    def _data(self, value):
        self.__parse_content_stream(BytesIO(value))


def _get_rectangle(this, name, defaults):
    retval = this.get(name)
    if isinstance(retval, RectangleObject):
        return retval
    if retval is None:
        for d in defaults:
            retval = this.get(d)
            if retval is not None:
                break
    if isinstance(retval, IndirectObject):
        retval = this.pdf.get_object(retval)
    retval = RectangleObject(retval)
    _set_rectangle(this, name, retval)
    return retval


def _set_rectangle(this, name, value):
    if not isinstance(name, NameObject):
        name = NameObject(name)
    this[name] = value


def _delete_rectangle(this, name):
    del this[name]


def _create_rotation_scaling_matrix(rotation, scale):
    rotation = math.radians(rotation)
    rotating = [[math.cos(rotation), math.sin(rotation), 0],
                [-math.sin(rotation), math.cos(rotation), 0],
                [0, 0, 1]]
    scaling = [[scale, 0, 0],
               [0, scale, 0],
               [0, 0, 1]]
    ctm = utils.matrix_multiply(rotating, scaling)
    return ctm


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
                retval = TextStringObject(decode_pdf_doc_encoding(string))
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


def is_plain_object(obj):
    return isinstance(obj, _PLAIN_OBJECTS)


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
_PLAIN_OBJECTS = (IndirectObject, NumberObject, NameObject, FloatObject, BooleanObject)
