# coding: utf-8
"""
A pure-Python PDF library with very minimal capabilities.  It was designed to
be able to split and merge PDF files by page, and that's about all it can do.
It may be a solid base for future PDF file work in Python.
"""
import io
import struct
from io import BufferedReader, BytesIO

import PyPDF.compound as _c
import PyPDF.utils as _u
import PyPDF.keys as _k
from PyPDF.generic import (
    NameObject, NumberObject, BooleanObject, TextStringObject,
    IndirectObject, ByteStringObject,
    DictionaryObject, ArrayObject,
    StreamObject, DocumentInformation, Destination,
    create_string_object, read_object,
)

_inheritable_page_attributes = (
    NameObject(_k.RESOURCES),
    NameObject(_k.MEDIA_BOX),
    NameObject(_k.CROP_BOX),
    NameObject(_k.ROTATE)
)


class PdfFileReader(object):
    def __init__(self, stream):
        """Initializes a PdfFileReader object.  This operation can take some time, as
        the PDF stream's cross-reference tables are read into memory.

        Stability: Added in v1.0, will exist for all v1.x releases.

        stream - An object that supports the standard read
                 and seek methods similar to a file object."""
        self._xref = {}
        self._xref_obj_stream = {}
        self._trailer = DictionaryObject()
        self._stream: BufferedReader = stream
        self._read_cross_reference()

        self._resolved_objects = {}
        self._named_dests = None
        self._override_encryption = False

        self._flattened_pages = []
        self._flatten(self._trailer[_k.ROOT].get_object()[_k.PAGES].get_object(), dict())

    def get_document_info(self):
        """Retrieves the PDF file's document information dictionary, if it exists.
        Note that some PDF files use metadata streams instead of docinfo
        dictionaries, and these metadata streams will not be accessed by this
        function.

        Stability: Added in v1.6, will exist for all future v1.x releases.

        return - Returns a DocumentInformation instance, or None if none exists."""
        if _k.INFO not in self._trailer:
            return None
        obj = self._trailer[_k.INFO]
        retval = DocumentInformation()
        retval.update(obj)
        return retval

    def get_xmp_metadata(self):
        """Retrieves XMP (Extensible Metadata Platform) data from the PDF document
        root.

        Stability: Added in v1.12, will exist for all future v1.x releases.
        @return Returns a {@link #generic.XmpInformation XmlInformation}
        instance that can be used to access XMP metadata from the document.
        Can also return None if no metadata was found on the document root."""
        try:
            self._override_encryption = True
            return self._trailer[_k.ROOT].get_xmp_metadata()
        finally:
            self._override_encryption = False

    def get_pages_count(self):
        """Calculates the number of pages in this PDF file.
        <p>
        Stability: Added in v1.0, will exist for all v1.x releases.
        @return Returns an integer."""
        return len(self._flattened_pages)

    def get_page(self, page_number):
        """Retrieves a page by number from this PDF file.

        Stability: Added in v1.0, will exist for all v1.x releases.
        @return Returns a {@link #PageObject PageObject} instance."""
        return self._flattened_pages[page_number]

    def get_named_destinations(self, tree=None, retval=None):
        """Retrieves the named destinations present in the document.

        Stability: Added in v1.10, will exist for all future v1.x releases.
        @return Returns a dict which maps names to {@link #Destination
        destinations}."""
        if retval is None:
            retval = {}
            catalog = self._trailer[_k.ROOT]

            # get the name tree
            if _k.DESTS in catalog:
                tree = catalog[_k.DESTS]
            elif _k.NAMES in catalog:
                names = catalog[_k.NAMES]
                if _k.DESTS in names:
                    tree = names[_k.DESTS]

        if tree is None:
            return retval

        if _k.KIDS in tree:
            # recurse down the tree
            for kid in tree[_k.KIDS]:
                self.get_named_destinations(kid.get_object(), retval)

        if _k.NAMES in tree:
            names = tree[_k.NAMES]
            for i in range(0, len(names), 2):
                key = names[i].get_object()
                val = names[i + 1].get_object()
                if isinstance(val, DictionaryObject) and b'/D' in val:
                    val = val[b'/D']
                dest = _build_destination(key, val)
                if dest is not None:
                    retval[key] = dest

        return retval

    def get_outlines(self, node=None, outlines=None):
        """Retrieves the document outline present in the document.

        Stability: Added in v1.10, will exist for all future v1.x releases.
        @return Returns a nested list of {@link #Destination destinations}."""
        if outlines is None:
            outlines = []
            catalog = self._trailer[_k.ROOT]

            # get the outline dictionary and named destinations
            if b'/Outlines' in catalog:
                lines = catalog[b'/Outlines']
                if b'/First' in lines:
                    node = lines[b'/First']
            self._named_dests = self.get_named_destinations()

        if node is None:
            return outlines

        # see if there are any more outlines
        while 1:
            outline = self._build_outline(node)
            if outline:
                outlines.append(outline)

            # check for sub-outlines
            if b'/First' in node:
                sub_outlines = []
                self.get_outlines(node[b'/First'], sub_outlines)
                if sub_outlines:
                    outlines.append(sub_outlines)

            if b'/Next' not in node:
                break
            node = node[b'/Next']

        return outlines

    def decrypt(self, password):
        """When using an encrypted / secured PDF file with the PDF Standard
        encryption handler, this function will allow the file to be decrypted.
        It checks the given password against the document's user password and
        owner password, and then stores the resulting decryption key if either
        password is correct.

        It does not matter which password was matched.  Both passwords provide
        the correct decryption key that will allow the document to be used with
        this library.

        Stability: Added in v1.8, will exist for all future v1.x releases.

        Return: 0 if the password failed, 1 if the password matched the user
        password, and 2 if the password matched the owner password.

        Throws: NotImplementedError Document uses an unsupported encryption
        method."""
        self._override_encryption = True
        try:
            return self._decrypt(password)
        finally:
            self._override_encryption = False

    def get_object(self, indirect_reference: IndirectObject):
        retval = self._resolved_objects.get(indirect_reference.generation, {}).get(indirect_reference.idnum, None)
        if retval is not None:
            return retval
        if indirect_reference.generation == 0 and indirect_reference.idnum in self._xref_obj_stream:
            # indirect reference to object in object stream
            # read the entire object stream into memory
            stmnum, idx = self._xref_obj_stream[indirect_reference.idnum]
            obj_stm = IndirectObject(stmnum, 0, self).get_object()
            assert obj_stm[_k.TYPE] == b'/ObjStm'
            assert idx < obj_stm[b'/N']
            stream_data = BytesIO(obj_stm.get_data())
            for i in range(obj_stm[b'/N']):
                objnum = NumberObject.read_from_stream(stream_data)
                _u.seek_token(stream_data)
                offset = NumberObject.read_from_stream(stream_data)
                _u.seek_token(stream_data)
                t = stream_data.tell()
                stream_data.seek(obj_stm[b'/First'] + offset, io.SEEK_SET)
                obj = read_object(stream_data, self)
                self._resolved_objects[0][objnum] = obj
                stream_data.seek(t, io.SEEK_SET)
            return self._resolved_objects[0][indirect_reference.idnum]
        start = self._xref[indirect_reference.generation][indirect_reference.idnum]
        self._stream.seek(start, io.SEEK_SET)
        idnum, generation = _read_object_header(self._stream)
        assert idnum == indirect_reference.idnum
        assert generation == indirect_reference.generation
        retval = read_object(self._stream, self)

        # override encryption is used for the /Encrypt dictionary
        if not self._override_encryption and self._is_encrypted:
            # if we don't have the encryption key:
            if self._decryption_key is None:
                raise Exception("file has not been decrypted")
            # otherwise, decrypt here...
            pack1 = struct.pack("<i", indirect_reference.idnum)[:3]
            pack2 = struct.pack("<i", indirect_reference.generation)[:2]
            key = _u.encrypt(self._decryption_key, pack1, pack2)
            retval = self._decrypt_object(retval, key)

        self._cache_indirect_object(generation, idnum, retval)
        return retval

    def _cache_indirect_object(self, generation, idnum, obj):
        if generation not in self._resolved_objects:
            self._resolved_objects[generation] = {}
        self._resolved_objects[generation][idnum] = obj

    def _read_cross_reference(self):
        # start at the end:
        self._stream.seek(-1, io.SEEK_END)
        line = ''
        while not line:
            line = _read_backward_for_line(self._stream)
        if not line[:5] == b'%%EOF':
            raise _u.PdfReadError("EOF marker not found")

        # find startxref entry - the location of the xref table
        line = _read_backward_for_line(self._stream)
        startxref = int(line)
        line = _read_backward_for_line(self._stream)
        if not line[:9] == b'startxref':
            raise _u.PdfReadError("Token 'startxref' not found")

        # read all cross reference tables and their trailers
        while 1:
            # load the xref table
            self._stream.seek(startxref, io.SEEK_SET)
            x = self._stream.read(1)
            if x in b'x':
                startxref = self._parse_standard_cross_reference_table()
                if startxref is None:
                    break
            elif x.isdigit():
                startxref = self._parse_1_5_cross_reference_table()
                if startxref is None:
                    break
            else:
                # bad xref character at startxref.  Let's see if we can find
                # the xref table nearby, as we've observed this error with an
                # off-by-one before.
                self._stream.seek(-11, io.SEEK_CUR)
                tmp = self._stream.read(20)
                xref_loc = tmp.find(b'xref')
                if xref_loc != -1:
                    startxref -= (10 - xref_loc)
                    continue
                else:
                    # no xref table found at specified location
                    assert False

    def _parse_standard_cross_reference_table(self):
        # standard cross-reference table
        ref = self._stream.read(4)
        if not ref[:3] == b'ref':
            raise _u.PdfReadError("xref table read error")
        _u.seek_token(self._stream)
        while 1:
            num = read_object(self._stream, self)
            _u.seek_token(self._stream)
            size = read_object(self._stream, self)
            _u.seek_token(self._stream)
            cnt = 0
            while cnt < size:
                line = self._stream.read(20)
                # It's very clear in section 3.4.3 of the PDF spec
                # that all cross-reference table lines are a fixed
                # 20 bytes.  However... some malformed PDF files
                # use a single character EOL without a preceeding
                # space.  Detect that case, and seek the stream
                # back one character.  (0-9 means we've bled into
                # the next xref entry, t means we've bled into the
                # text "trailer"):
                if line[-1] in b'0123456789t':
                    self._stream.seek(-1, io.SEEK_CUR)
                offset, generation = line[:16].split(b' ')
                offset, generation = int(offset), int(generation)
                if generation not in self._xref:
                    self._xref[generation] = {}
                if num in self._xref[generation]:
                    # It really seems like we should allow the last
                    # xref table in the file to override previous
                    # ones. Since we read the file backwards, assume
                    # any existing key is already set correctly.
                    pass
                else:
                    self._xref[generation][num] = offset
                cnt += 1
                num += 1
            _u.seek_token(self._stream)
            trailertag = self._stream.read(7)
            if not trailertag == b'trailer':
                # more xrefs!
                self._stream.seek(-7, io.SEEK_CUR)
            else:
                break
        _u.seek_token(self._stream)
        new_trailer = read_object(self._stream, self)
        for key, value in new_trailer.items():
            if key not in self._trailer:
                self._trailer[key] = value
        if b'/Prev' in new_trailer:
            return new_trailer[b'/Prev']
        else:
            return None

    def _parse_1_5_cross_reference_table(self):
        # PDF 1.5+ Cross-Reference Stream
        self._stream.seek(-1, io.SEEK_CUR)
        idnum, generation = _read_object_header(self._stream)
        xrefstream = read_object(self._stream, self)
        _u.debug(xrefstream)
        assert xrefstream[_k.TYPE] == b'/XRef'
        self._cache_indirect_object(generation, idnum, xrefstream)
        stream_data = BytesIO(xrefstream.get_data())
        idx_pairs = xrefstream.get(b'/Index', [0, xrefstream.get(_k.SIZE)])
        entry_sizes = xrefstream.get(b'/W')
        for num, size in _generate_pairs(idx_pairs):
            cnt = 0
            xref_type = None
            byte_offset = None
            objstr_num = None
            obstr_idx = None
            while cnt < size:
                for i in range(len(entry_sizes)):
                    d = stream_data.read(entry_sizes[i])
                    di = _convert_to_int(d, entry_sizes[i])
                    if i == 0:
                        xref_type = di
                    elif i == 1:
                        if xref_type == 0:
                            _next_free_object = di
                        elif xref_type == 1:
                            byte_offset = di
                        elif xref_type == 2:
                            objstr_num = di
                    elif i == 2:
                        if xref_type == 0:
                            _next_generation = di
                        elif xref_type == 1:
                            generation = di
                        elif xref_type == 2:
                            obstr_idx = di
                if xref_type == 0:
                    pass
                elif xref_type == 1:
                    if generation not in self._xref:
                        self._xref[generation] = {}
                    if num not in self._xref[generation]:
                        self._xref[generation][num] = byte_offset
                elif xref_type == 2:
                    if num not in self._xref_obj_stream:
                        self._xref_obj_stream[num] = [objstr_num, obstr_idx]
                cnt += 1
                num += 1
        trailer_keys = _k.ROOT, _k.ENCRYPT, _k.INFO, _k.ID
        for key in trailer_keys:
            if key in xrefstream and key not in self._trailer:
                self._trailer[NameObject(key)] = xrefstream.raw_get(key)
        if b'/Prev' in xrefstream:
            return xrefstream[b'/Prev']
        else:
            return None

    def _decrypt(self, password):
        encrypt = self._trailer[_k.ENCRYPT].get_object()
        if not encrypt[b'/Filter'] == b'/Standard':
            raise NotImplementedError("only Standard PDF encryption handler is available")
        if not (encrypt[b'/V'] in (1, 2)):
            raise NotImplementedError("only algorithm code 1 and 2 are supported")
        user_password, key = self._authenticate_user_password(password)
        if user_password:
            self._decryption_key = key
            return 1
        else:
            rev = encrypt[b'/R'].get_object()
            if rev == 2:
                keylen = 5
            else:
                keylen = encrypt[b'/Length'].get_object() / 8
            key = _u.algorithm_33_1(password, rev, keylen)
            real_o = encrypt[b'/O'].get_object()
            if rev == 2:
                userpass = _u.rc4_encrypt(key, real_o)
            else:
                val = real_o
                for i in range(19, -1, -1):
                    new_key = ''
                    for j in range(len(key)):
                        new_key += chr(ord(key[j:j + 1]) ^ i)
                    val = _u.rc4_encrypt(new_key, val)
                userpass = val
            owner_password, key = self._authenticate_user_password(userpass)
            if owner_password:
                self._decryption_key = key
                return 2
        return 0

    def _authenticate_user_password(self, password):
        encrypt = self._trailer[_k.ENCRYPT].get_object()
        rev = encrypt[b'/R'].get_object()
        owner_entry = encrypt[b'/O'].get_object().original_bytes
        p_entry = encrypt[b'/P'].get_object()
        id_entry = self._trailer[_k.ID].get_object()
        id1_entry = id_entry[0].get_object()
        u = None
        key = None
        if rev == 2:
            u, key = _u.algorithm_34(password, owner_entry, p_entry, id1_entry)
        elif rev >= 3:
            u, key = _u.algorithm_35(
                password, rev,
                encrypt[b'/Length'].get_object() / 8, owner_entry,
                p_entry, id1_entry,
                encrypt.get(b'/EncryptMetadata', BooleanObject(False)).get_object()
            )
        real_u = encrypt[b'/U'].get_object().original_bytes
        return u == real_u, key

    def _decrypt_object(self, obj, key):
        if isinstance(obj, ByteStringObject) or isinstance(obj, TextStringObject):
            obj = create_string_object(_u.rc4_encrypt(key, obj.original_bytes))
        elif isinstance(obj, StreamObject):
            obj.set_data(_u.rc4_encrypt(key, obj.get_data()))
        elif isinstance(obj, DictionaryObject):
            for dictkey, value in obj.items():
                obj[dictkey] = self._decrypt_object(value, key)
        elif isinstance(obj, ArrayObject):
            for i in range(len(obj)):
                obj[i] = self._decrypt_object(obj[i], key)
        return obj

    def _build_outline(self, node):
        dest, title, outline = None, None, None

        if b'/A' in node and b'/Title' in node:
            # Action, section 8.5 (only type GoTo supported)
            title = node[b'/Title']
            action = node[b'/A']
            if action[b'/S'] == b'/GoTo':
                dest = action[b'/D']
        elif b'/Dest' in node and b'/Title' in node:
            # Destination, section 8.2.1
            title = node[b'/Title']
            dest = node[b'/Dest']

        # if destination found, then create outline
        if dest:
            if isinstance(dest, ArrayObject):
                outline = _build_destination(title, dest)
            # FIXME elif dest in isinstance(dest, unicode) and self._namedDests:
            elif dest in isinstance(dest, str) and self._named_dests:
                outline = self._named_dests[dest]
                outline[NameObject(b'/Title')] = title
            else:
                raise _u.PdfReadError("Unexpected destination %r" % dest)
        return outline

    def _flatten(self, pages, inherit, indirect_ref=None):
        target_type = pages[_k.TYPE]
        if target_type == _k.PAGES:
            for attr in _inheritable_page_attributes:
                if attr in pages:
                    inherit[attr] = pages[attr]
            for page in pages[_k.KIDS]:
                self._flatten(
                    page.get_object(), inherit,
                    indirect_ref=page if isinstance(page, IndirectObject) else None,
                )
        elif target_type == _k.PAGE:
            for attr, value in inherit.items():
                # if the page has it's own value, it does not inherit the
                # parent's value:
                if attr not in pages:
                    pages[attr] = value
            page_obj = _c.PageObject(self, indirect_ref)
            page_obj.update(pages)  # FIXME XXX The page absorbed all pages attributes?
            self._flattened_pages.append(page_obj)

    @property
    def _is_encrypted(self):
        return _k.ENCRYPT in self._trailer


def _convert_to_int(d, size):
    if size > 8:
        raise _u.PdfReadError("invalid size in convertToInt")
    d = "\x00\x00\x00\x00\x00\x00\x00\x00" + d
    d = d[-8:]
    return struct.unpack(">q", d)[0]


def _generate_pairs(array):
    i = 0
    while True:
        yield array[i], array[i + 1]
        i += 2
        if (i + 1) >= len(array):
            break


def _build_destination(title, array):
    page, typ = array[0:2]
    array = array[2:]
    return Destination(title, page, typ, *array)


def _read_object_header(stream):
    """Should never be necessary to read out whitespace, since the
    cross-reference table should put us in the right spot to read the
    object header.  In reality... some files have stupid cross reference
    tables that are off by whitespace bytes."""
    _u.seek_token(stream)
    idnum = _u.read_until_whitespace(stream)
    generation = _u.read_until_whitespace(stream)
    _obj = stream.read(3)
    _u.seek_token(stream)
    return int(idnum), int(generation)


def _read_backward_for_line(stream: BufferedReader):
    line = b''
    while True:
        x = stream.read(1)
        stream.seek(-2, io.SEEK_CUR)
        if x in b'\n\r':
            while x in b'\n\r':
                x = stream.read(1)
                stream.seek(-2, io.SEEK_CUR)
            stream.seek(1, io.SEEK_CUR)
            break
        else:
            line = x + line
    return line
