import io
import math
import PyPDF.utils as utils
import PyPDF.keys as _k
from PyPDF.generic import (
    NameObject, NumberObject, TextStringObject, FloatObject, NullObject, IndirectObject,
    DictionaryObject, ArrayObject,
    DecodedStreamObject,
    read_object,
)

RESOURCES_KEY = b'/Resources'
_IMAGE_KEY = b'INLINE IMAGE'
_CONTENT_KEY = b'/Contents'


def _create_rectangle_accessor(name, fallback):
    return property(
        lambda self: _get_rectangle(self, name, fallback),
        lambda self, value: _set_rectangle(self, name, value),
        lambda self: _delete_rectangle(self, name)
    )


class PageObject(DictionaryObject):
    """
    This class represents a single page within a PDF file.  Typically this object
    will be created by accessing the PdfFileReader.get_page function of the
    PdfFileReader class, but it is also possible to create an empty page
    with the create_blank_page static method.

    Constructor arguments:
    _pdf -- PDF file the page belongs to (optional, defaults to None).
    """

    def __init__(self, parent=None, indirect_ref=None):
        DictionaryObject.__init__(self)
        self.parent = parent
        # Stores the original indirect reference to this object in its source PDF
        self.indirect_ref = indirect_ref

    def get_contents(self):
        """Returns the /Contents object, or None if it doesn't exist.
        /Contents is optionnal, as described in PDF Reference  7.7.3.3"""
        if _CONTENT_KEY in self:
            return self[_CONTENT_KEY].get_object()
        else:
            return None

    def merge_page(self, page2, page2transformation=None):
        """Merges the content streams of two pages into one. Resource
        references (i.e. fonts) are maintained from both pages. The
        mediabox/cropbox/etc of this page are not altered. The parameter page's
        content stream will be added to the end of this page's content stream,
        meaning that it will be drawn after, or "on top" of this page.

        page2 - An instance of {@link #PageObject PageObject} to be merged
                into this one.
        page2transformation - A fuction which applies a transformation to
                              the content stream of page2. Takes: page2
                              contents stream. Must return: new contents
                              stream. If omitted, the content stream will
                              not be modified."""
        # First we work on merging the resource dictionaries.  This allows us
        # to find out what symbols in the content streams we might need to
        # rename.
        new_resources = DictionaryObject()
        rename = {}
        original_resources = self[RESOURCES_KEY].get_object()
        page2_resources = page2[RESOURCES_KEY].get_object()

        for res in b'/ExtGState', b'/Font', b'/XObject', b'/ColorSpace', b'/Pattern', b'/Shading', b'/Properties':
            new, newrename = _merge_resources(original_resources, page2_resources, res)
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
            new_content_array.append(_push_pop_graphics_state(original_content, self.parent))

        page2_content = page2.get_contents()
        if page2_content is not None:
            if page2transformation is not None:
                page2_content = page2transformation(page2_content)
            page2_content = _content_stream_rename(
                page2_content, rename, self.parent)
            page2_content = _push_pop_graphics_state(page2_content, self.parent)
            new_content_array.append(page2_content)

        utils.debug('-' * 16)
        self[NameObject(_CONTENT_KEY)] = _ContentStreamObject(new_content_array, self.parent)
        self[NameObject(RESOURCES_KEY)] = new_resources

    def rotate_clockwise(self, angle):
        """Rotates a page clockwise by increments of 90 degrees.

        Stability: Added in v1.1, will exist for all future v1.x releases.
        @param angle Angle to rotate the page.  Must be an increment of 90 deg."""
        assert angle % 90 == 0
        self._rotate(angle)
        return self

    def rotate_counter_clockwise(self, angle):
        """Rotates a page counter-clockwise by increments of 90 degrees.

        Stability: Added in v1.1, will exist for all future v1.x releases.
        @param angle Angle to rotate the page.  Must be an increment of 90 deg."""
        assert angle % 90 == 0
        self._rotate(-angle)
        return self

    def _rotate(self, angle):
        current_angle = self.get(b'/Rotate', 0)
        self[NameObject(b'/Rotate')] = NumberObject(current_angle + angle)

    def merge_transformed_page(self, page2, ctm):
        """This is similar to mergePage, but a transformation matrix is
        applied to the merged stream.

        page2 - An instance of {@link #PageObject PageObject} to be merged.
        ctm   - A 6 elements tuple containing the operands of the
                     transformation matrix"""
        self.merge_page(page2, lambda page2_content: _add_transformation_matrix(
            page2_content, page2.parent, ctm
        ))

    def merge_scaled_page(self, page2, factor):
        """This is similar to mergePage, but the stream to be merged is scaled
        by appling a transformation matrix.

        page2  - An instance of {@link #PageObject PageObject} to be merged.
        factor - The scaling factor"""
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
            new_content = _add_transformation_matrix(
                original_content, self.parent, ctm)
            new_content = _push_pop_graphics_state(new_content, self.parent)
            self[NameObject(_CONTENT_KEY)] = new_content

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
            if not isinstance(content, _ContentStreamObject):
                content = _ContentStreamObject(content, self.parent)
            self[NameObject(_CONTENT_KEY)] = content.flate_encode()

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
        content = self[_CONTENT_KEY].get_object()
        if not isinstance(content, _ContentStreamObject):
            content = _ContentStreamObject(content, self.parent)
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


class _ContentStreamObject(DecodedStreamObject):
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
                utils.debug(s)
                data += s.get_object().get_data()
            stream = io.BytesIO(data)
        else:
            stream = io.BytesIO(stream.get_data())
        # assert stream is None
        self.__parse_content_stream(stream)
        utils.debug(_pdf)
        utils.stacktrace_debug()

    def __parse_content_stream(self, stream):
        stream.seek(0, io.SEEK_SET)
        operands = []
        while True:
            peek = utils.read_non_whitespace(stream)
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
                    ii = self.__read_inline_image(stream)
                    self.operations.append((ii, _IMAGE_KEY))
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

    def __read_inline_image(self, stream):
        # begin reading just after the "BI" - begin image
        # first read the dictionary of settings.
        settings = DictionaryObject()
        while True:
            tok = utils.seek_token(stream)
            if tok == b'I':
                # "ID" - begin of image data
                break
            key = read_object(stream, self.pdf)
            utils.seek_token(stream)
            value = read_object(stream, self.pdf)
            settings[key] = value
        # left at beginning of ID
        tmp = stream.read(3)
        assert tmp[:2] == b'ID'
        data = _read_image_data(stream)
        utils.debug(len(data))
        utils.seek_token(stream)
        return {b'settings': settings, b'data': data}

    @property
    def _data(self):
        newdata = io.BytesIO()
        for operands, operator in self.operations:
            if operator == _IMAGE_KEY:
                newdata.write(b'BI')
                dicttext = io.BytesIO()
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
        self.__parse_content_stream(io.BytesIO(value))


class RectangleObject(ArrayObject):
    def __init__(self, arr):
        # must have four points
        assert len(arr) == 4
        # automatically convert arr[x] into NumberObject(arr[x]) if necessary
        ArrayObject.__init__(self, [_ensure_is_number(x) for x in arr])

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
        self[0], self[1] = [_ensure_is_number(x) for x in value]

    def set_lower_right(self, value):
        self[2], self[1] = [_ensure_is_number(x) for x in value]

    def set_upper_left(self, value):
        self[0], self[3] = [_ensure_is_number(x) for x in value]

    def set_upper_right(self, value):
        self[2], self[3] = [_ensure_is_number(x) for x in value]

    def get_width(self):
        return self.get_upper_right_x() - self.get_lower_left_x()

    def get_height(self):
        return self.get_upper_right_y() - self.get_lower_left_x()


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
    page.__setitem__(NameObject(_k.TYPE_KEY), NameObject(b'/Page'))
    page.__setitem__(NameObject(b'/Parent'), NullObject())
    page.__setitem__(NameObject(RESOURCES_KEY), DictionaryObject())
    if width is None or height is None:
        if _pdf is not None and _pdf.get_pages_count() > 0:
            lastpage = _pdf.get_page(_pdf.get_pages_count() - 1)
            width = lastpage.media_box.get_width()
            height = lastpage.media_box.get_height()
        else:
            raise utils.PageSizeNotDefinedError()
    page.__setitem__(NameObject(b'/MediaBox'), RectangleObject([0, 0, width, height]))
    return page


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
        retval = this.parent.get_object(retval)
    retval = RectangleObject(retval)
    _set_rectangle(this, name, retval)
    return retval


def _set_rectangle(this, name, value):
    if not isinstance(name, NameObject):
        name = NameObject(name)
    this[name] = value


def _delete_rectangle(this, name):
    del this[name]


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


def _add_transformation_matrix(contents, _pdf, ctm):
    """adds transformation matrix at the beginning of the given contents stream."""
    a, b, c, d, e, f = ctm
    contents = _ContentStreamObject(contents, _pdf)
    contents.operations.insert(0, [[FloatObject(a), FloatObject(b),
                                    FloatObject(c), FloatObject(d), FloatObject(e),
                                    FloatObject(f)], " cm"])
    return contents


def _push_pop_graphics_state(contents, _pdf):
    """adds a graphics state "push" and "pop" to the beginning and end
    of a content stream.  This isolates it from changes such as
    transformation matricies."""
    stream = _ContentStreamObject(contents, _pdf)
    stream.operations.insert(0, [[], b'q'])
    stream.operations.append([[], b'Q'])
    return stream


def _content_stream_rename(stream, rename, _pdf):
    if not rename:
        return stream
    stream = _ContentStreamObject(stream, _pdf)
    for operands, operator in stream.operations:
        for i in range(len(operands)):
            op = operands[i]
            if isinstance(op, NameObject):
                operands[i] = rename.get(op, op)
    return stream


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


def _ensure_is_number(value):
    if not isinstance(value, (NumberObject, FloatObject)):
        value = FloatObject(value)
    return value


def _read_image_data(stream):
    data = b''
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
    return data
