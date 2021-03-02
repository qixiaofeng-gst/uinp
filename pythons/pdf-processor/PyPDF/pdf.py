# coding: utf-8
"""
A pure-Python PDF library with very minimal capabilities.  It was designed to
be able to split and merge PDF files by page, and that's about all it can do.
It may be a solid base for future PDF file work in Python.
"""
import io
import struct
import time
import random
from hashlib import md5
from io import BufferedReader, BytesIO

import PyPDF.utils as utils
from PyPDF.generic import (
    NameObject, DictionaryObject, NumberObject, ArrayObject, BooleanObject, PageObject,
    IndirectObject, ByteStringObject, StreamObject, TextStringObject, DocumentInformation, Destination,
    create_string_object, read_object, is_plain_object, create_blank_page,
)
from PyPDF.utils import (
    seek_token, read_until_whitespace,
)


class PdfFileReader(object):
    def __init__(self, stream):
        """Initializes a PdfFileReader object.  This operation can take some time, as
        the PDF stream's cross-reference tables are read into memory.

        Stability: Added in v1.0, will exist for all v1.x releases.

        stream - An object that supports the standard read
                 and seek methods similar to a file object."""
        self.xref = {}
        self._xref_obj_stream = {}
        self.trailer = None
        self._named_dests = None
        self._flattened_pages = None
        self._resolved_objects = {}
        self.read(stream)
        self._stream: BufferedReader = stream
        self._override_encryption = False

    def get_document_info(self):
        """Retrieves the PDF file's document information dictionary, if it exists.
        Note that some PDF files use metadata streams instead of docinfo
        dictionaries, and these metadata streams will not be accessed by this
        function.

        Stability: Added in v1.6, will exist for all future v1.x releases.

        return - Returns a DocumentInformation instance, or None if none exists."""
        if b'/Info' not in self.trailer:
            return None
        obj = self.trailer[b'/Info']
        retval = DocumentInformation()
        retval.update(obj)
        return retval

    ##
    # Retrieves XMP (Extensible Metadata Platform) data from the PDF document
    # root.
    # <p>
    # Stability: Added in v1.12, will exist for all future v1.x releases.
    # @return Returns a {@link #generic.XmpInformation XmlInformation}
    # instance that can be used to access XMP metadata from the document.
    # Can also return None if no metadata was found on the document root.
    def get_xmp_metadata(self):
        try:
            self._override_encryption = True
            return self.trailer[b'/Root'].get_xmp_metadata()
        finally:
            self._override_encryption = False

    ##
    # Calculates the number of pages in this PDF file.
    # <p>
    # Stability: Added in v1.0, will exist for all v1.x releases.
    # @return Returns an integer.
    def get_pages_count(self):
        if self._flattened_pages is None:
            self._flatten()
        return len(self._flattened_pages)

    ##
    # Retrieves a page by number from this PDF file.
    # <p>
    # Stability: Added in v1.0, will exist for all v1.x releases.
    # @return Returns a {@link #PageObject PageObject} instance.
    def get_page(self, page_number):
        # ensure that we're not trying to access an encrypted PDF
        # assert not self.trailer.has_key("/Encrypt")
        if self._flattened_pages is None:
            self._flatten()
        return self._flattened_pages[page_number]

    ##
    # Retrieves the named destinations present in the document.
    # <p>
    # Stability: Added in v1.10, will exist for all future v1.x releases.
    # @return Returns a dict which maps names to {@link #Destination
    # destinations}.
    def get_named_destinations(self, tree=None, retval=None):
        if retval is None:
            retval = {}
            catalog = self.trailer[b'/Root']

            # get the name tree
            if b'/Dests' in catalog:
                tree = catalog[b'/Dests']
            elif b'/Names' in catalog:
                names = catalog[b'/Names']
                if b'/Dests' in names:
                    tree = names[b'/Dests']

        if tree is None:
            return retval

        if b'/Kids' in tree:
            # recurse down the tree
            for kid in tree[b'/Kids']:
                self.get_named_destinations(kid.get_object(), retval)

        if b'/Names' in tree:
            names = tree[b'/Names']
            for i in range(0, len(names), 2):
                key = names[i].get_object()
                val = names[i + 1].get_object()
                if isinstance(val, DictionaryObject) and b'/D' in val:
                    val = val[b'/D']
                dest = build_destination(key, val)
                if dest is not None:
                    retval[key] = dest

        return retval

    ##
    # Retrieves the document outline present in the document.
    # <p>
    # Stability: Added in v1.10, will exist for all future v1.x releases.
    # @return Returns a nested list of {@link #Destination destinations}.
    def get_outlines(self, node=None, outlines=None):
        if outlines is None:
            outlines = []
            catalog = self.trailer[b'/Root']

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

    def get_object(self, indirect_reference):
        retval = self._resolved_objects.get(indirect_reference.generation, {}).get(indirect_reference.idnum, None)
        if retval is not None:
            return retval
        if indirect_reference.generation == 0 and indirect_reference.idnum in self._xref_obj_stream:
            # indirect reference to object in object stream
            # read the entire object stream into memory
            stmnum, idx = self._xref_obj_stream[indirect_reference.idnum]
            obj_stm = IndirectObject(stmnum, 0, self).get_object()
            assert obj_stm[b'/Type'] == b'/ObjStm'
            assert idx < obj_stm[b'/N']
            stream_data = BytesIO(obj_stm.get_data())
            for i in range(obj_stm[b'/N']):
                objnum = NumberObject.read_from_stream(stream_data)
                seek_token(stream_data)
                offset = NumberObject.read_from_stream(stream_data)
                seek_token(stream_data)
                t = stream_data.tell()
                stream_data.seek(obj_stm[b'/First'] + offset, io.SEEK_SET)
                obj = read_object(stream_data, self)
                self._resolved_objects[0][objnum] = obj
                stream_data.seek(t, io.SEEK_SET)
            return self._resolved_objects[0][indirect_reference.idnum]
        start = self.xref[indirect_reference.generation][indirect_reference.idnum]
        self._stream.seek(start, io.SEEK_SET)
        idnum, generation = read_object_header(self._stream)
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
            key = _encrypt(self._decryption_key, pack1, pack2)
            retval = self._decrypt_object(retval, key)

        self.cache_indirect_object(generation, idnum, retval)
        return retval

    def cache_indirect_object(self, generation, idnum, obj):
        if generation not in self._resolved_objects:
            self._resolved_objects[generation] = {}
        self._resolved_objects[generation][idnum] = obj

    def read(self, stream):
        # start at the end:
        stream.seek(-1, io.SEEK_END)
        line = ''
        while not line:
            line = read_next_end_line(stream)
        if not line[:5] == b'%%EOF':
            raise utils.PdfReadError("EOF marker not found")

        # find startxref entry - the location of the xref table
        line = read_next_end_line(stream)
        startxref = int(line)
        line = read_next_end_line(stream)
        if not line[:9] == b'startxref':
            raise utils.PdfReadError("Token 'startxref' not found")

        # read all cross reference tables and their trailers
        self.xref = {}
        self._xref_obj_stream = {}
        self.trailer = DictionaryObject()
        while 1:
            # load the xref table
            stream.seek(startxref, io.SEEK_SET)
            x = stream.read(1)
            if x in b'x':
                startxref = _parse_standard_cross_reference_table(stream, self)
                if startxref is None:
                    break
            elif x.isdigit():
                # PDF 1.5+ Cross-Reference Stream
                stream.seek(-1, io.SEEK_CUR)
                idnum, generation = read_object_header(stream)
                xrefstream = read_object(stream, self)
                utils.debug(xrefstream)
                assert xrefstream[b'/Type'] == b'/XRef'
                self.cache_indirect_object(generation, idnum, xrefstream)
                stream_data = BytesIO(xrefstream.get_data())
                idx_pairs = xrefstream.get(b'/Index', [0, xrefstream.get(b'/Size')])
                entry_sizes = xrefstream.get(b'/W')
                for num, size in generate_pairs(idx_pairs):
                    cnt = 0
                    xref_type = None
                    byte_offset = None
                    objstr_num = None
                    obstr_idx = None
                    while cnt < size:
                        for i in range(len(entry_sizes)):
                            d = stream_data.read(entry_sizes[i])
                            di = convert_to_int(d, entry_sizes[i])
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
                            if generation not in self.xref:
                                self.xref[generation] = {}
                            if num not in self.xref[generation]:
                                self.xref[generation][num] = byte_offset
                        elif xref_type == 2:
                            if num not in self._xref_obj_stream:
                                self._xref_obj_stream[num] = [objstr_num, obstr_idx]
                        cnt += 1
                        num += 1
                trailer_keys = b'/Root', b'/Encrypt', b'/Info', b'/ID'
                for key in trailer_keys:
                    if key in xrefstream and key not in self.trailer:
                        self.trailer[NameObject(key)] = xrefstream.raw_get(key)
                if b'/Prev' in xrefstream:
                    startxref = xrefstream[b'/Prev']
                else:
                    break
            else:
                # bad xref character at startxref.  Let's see if we can find
                # the xref table nearby, as we've observed this error with an
                # off-by-one before.
                stream.seek(-11, io.SEEK_CUR)
                tmp = stream.read(20)
                xref_loc = tmp.find(b'xref')
                if xref_loc != -1:
                    startxref -= (10 - xref_loc)
                    continue
                else:
                    # no xref table found at specified location
                    assert False

    ##
    # When using an encrypted / secured PDF file with the PDF Standard
    # encryption handler, this function will allow the file to be decrypted.
    # It checks the given password against the document's user password and
    # owner password, and then stores the resulting decryption key if either
    # password is correct.
    # <p>
    # It does not matter which password was matched.  Both passwords provide
    # the correct decryption key that will allow the document to be used with
    # this library.
    # <p>
    # Stability: Added in v1.8, will exist for all future v1.x releases.
    #
    # @return 0 if the password failed, 1 if the password matched the user
    # password, and 2 if the password matched the owner password.
    #
    # @exception NotImplementedError Document uses an unsupported encryption
    # method.
    def decrypt(self, password):
        self._override_encryption = True
        try:
            return self._decrypt(password)
        finally:
            self._override_encryption = False

    def _decrypt(self, password):
        encrypt = self.trailer[b'/Encrypt'].get_object()
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
            key = _alg33_1(password, rev, keylen)
            real_o = encrypt[b'/O'].get_object()
            if rev == 2:
                userpass = utils.rc4_encrypt(key, real_o)
            else:
                val = real_o
                for i in range(19, -1, -1):
                    new_key = ''
                    for j in range(len(key)):
                        new_key += chr(ord(key[j:j + 1]) ^ i)
                    val = utils.rc4_encrypt(new_key, val)
                userpass = val
            owner_password, key = self._authenticate_user_password(userpass)
            if owner_password:
                self._decryption_key = key
                return 2
        return 0

    def _authenticate_user_password(self, password):
        encrypt = self.trailer[b'/Encrypt'].get_object()
        rev = encrypt[b'/R'].get_object()
        owner_entry = encrypt[b'/O'].get_object().original_bytes
        p_entry = encrypt[b'/P'].get_object()
        id_entry = self.trailer[b'/ID'].get_object()
        id1_entry = id_entry[0].get_object()
        u = None
        key = None
        if rev == 2:
            u, key = _alg34(password, owner_entry, p_entry, id1_entry)
        elif rev >= 3:
            u, key = _alg35(password, rev,
                            encrypt[b'/Length'].get_object() / 8, owner_entry,
                            p_entry, id1_entry,
                            encrypt.get(b'/EncryptMetadata', BooleanObject(False)).get_object())
        real_u = encrypt[b'/U'].get_object().original_bytes
        return u == real_u, key

    def _decrypt_object(self, obj, key):
        if isinstance(obj, ByteStringObject) or isinstance(obj, TextStringObject):
            obj = create_string_object(utils.rc4_encrypt(key, obj.original_bytes))
        elif isinstance(obj, StreamObject):
            obj.set_data(utils.rc4_encrypt(key, obj.get_data()))
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
                outline = build_destination(title, dest)
            # FIXME elif dest in isinstance(dest, unicode) and self._namedDests:
            elif dest in isinstance(dest, str) and self._named_dests:
                outline = self._named_dests[dest]
                outline[NameObject(b'/Title')] = title
            else:
                raise utils.PdfReadError("Unexpected destination %r" % dest)
        return outline

    def _flatten(self, pages=None, inherit=None, indirect_ref=None):
        inheritable_page_attributes = (
            NameObject(b'/Resources'), NameObject(b'/MediaBox'),
            NameObject(b'/CropBox'), NameObject(b'/Rotate')
        )
        if inherit is None:
            inherit = dict()
        if pages is None:
            self._flattened_pages = []
            catalog = self.trailer[b'/Root'].get_object()
            pages = catalog[b'/Pages'].get_object()
        t = pages[b'/Type']
        if t == b'/Pages':
            for attr in inheritable_page_attributes:
                if attr in pages:
                    inherit[attr] = pages[attr]
            for page in pages[b'/Kids']:
                addt = {}
                if isinstance(page, IndirectObject):
                    addt['indirect_ref'] = page
                self._flatten(page.get_object(), inherit, **addt)
        elif t == b'/Page':
            for attr, value in inherit.items():
                # if the page has it's own value, it does not inherit the
                # parent's value:
                if attr not in pages:
                    pages[attr] = value
            page_obj = PageObject(self, indirect_ref)
            page_obj.update(pages)
            self._flattened_pages.append(page_obj)

    @property
    def _is_encrypted(self):
        return b'/Encrypt' in self.trailer


def generate_pairs(array):
    i = 0
    while True:
        yield array[i], array[i + 1]
        i += 2
        if (i + 1) >= len(array):
            break


def build_destination(title, array):
    page, typ = array[0:2]
    array = array[2:]
    return Destination(title, page, typ, *array)


def read_object_header(stream):
    """Should never be necessary to read out whitespace, since the
    cross-reference table should put us in the right spot to read the
    object header.  In reality... some files have stupid cross reference
    tables that are off by whitespace bytes."""
    seek_token(stream)
    idnum = read_until_whitespace(stream)
    generation = read_until_whitespace(stream)
    _obj = stream.read(3)
    seek_token(stream)
    return int(idnum), int(generation)


def read_next_end_line(stream: BufferedReader):
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
    utils.debug(line)
    return line


class PdfFileWriter(object):
    """
    This class supports writing PDF files out, given pages produced
    by another class (typically PdfFileReader).
    """

    def __init__(self):
        self._id = None
        self._encrypt = None
        self._encrypt_key = None
        self._stack = []
        self._objects = []  # array of indirect objects

        # The root of our page tree node.
        pages = DictionaryObject()
        pages.update({
            NameObject(b'/Type'): NameObject(b'/Pages'),
            NameObject(b'/Count'): NumberObject(0),
            NameObject(b'/Kids'): ArrayObject(),
        })
        self._pages = self._add_object(pages)

        # info object
        info = DictionaryObject()
        info.update({
            NameObject(b'/Producer'): create_string_object(b'PyPDF - Refactored by xiaofeng.qi')
        })
        self._info = self._add_object(info)

        # root object
        root = DictionaryObject()
        root.update({
            NameObject(b'/Type'): NameObject(b'/Catalog'),
            NameObject(b'/Pages'): self._pages,
        })
        self._root = self._add_object(root)

    def get_object(self, ido):
        if not ido.pdf == self:
            raise ValueError("PyPDF must be self")
        return self._objects[ido.idnum - 1]

    def add_page(self, page):
        self._add_page(page, list.append)

    def insert_page(self, page, index=0):
        self._add_page(page, lambda l, p: l.insert(index, p))

    def get_page(self, page_number):
        """Retrieves a page by number from this PDF file."""
        pages = self._pages.get_object()
        return pages[b'/Kids'][page_number].get_object()

    def get_pages_count(self):
        pages = self._pages.get_object()
        return int(pages[NameObject(b'/Count')])

    def add_blank_page(self, width=None, height=None):
        page = create_blank_page(self, width, height)
        self.add_page(page)
        return page

    def insert_blank_page(self, width=None, height=None, index=0):
        if width is None or height is None and self.get_pages_count() > index:
            oldpage = self.get_page(index)
            width = oldpage.media_box.get_width()
            height = oldpage.media_box.get_height()
        page = create_blank_page(self, width, height)
        self.insert_page(page, index)
        return page

    def encrypt(self, user_pwd, owner_pwd=None, use_128bit=True):
        """Encrypt this PDF file with the PDF Standard encryption handler.

        user_pwd - The "user password", which allows for opening and reading
                the PDF file with the restrictions provided.
        owner_pwd - The "owner password", which allows for opening the PDF
                files without any restrictions.  By default, the owner password is the
                same as the user password.
        use_128bit - Boolean argument as to whether to use 128bit
                encryption.  When false, 40bit encryption will be used.  By default, this
                flag is on."""
        if owner_pwd is None:
            owner_pwd = user_pwd
        if use_128bit:
            v = 2
            rev = 3
            keylen = 128 / 8
        else:
            v = 1
            rev = 2
            keylen = 40 / 8
        # permit everything:
        p = -1
        o = ByteStringObject(_alg33(owner_pwd, user_pwd, rev, keylen))
        id_1 = md5(bytes(repr(time.time()), utils.ENCODING_UTF8)).digest()
        id_2 = md5(bytes(repr(random.random()), utils.ENCODING_UTF8)).digest()
        self._id = ArrayObject((ByteStringObject(id_1), ByteStringObject(id_2)))
        if rev == 2:
            u, key = _alg34(user_pwd, o, p, id_1)
        else:
            assert rev == 3
            u, key = _alg35(user_pwd, rev, keylen, o, p, id_1, False)
        encrypt = DictionaryObject()
        encrypt[NameObject(b'/Filter')] = NameObject(b'/Standard')
        encrypt[NameObject(b'/V')] = NumberObject(v)
        if v == 2:
            encrypt[NameObject(b'/Length')] = NumberObject(keylen * 8)
        encrypt[NameObject(b'/R')] = NumberObject(rev)
        encrypt[NameObject(b'/O')] = ByteStringObject(o)
        encrypt[NameObject(b'/U')] = ByteStringObject(u)
        encrypt[NameObject(b'/P')] = NumberObject(p)
        self._encrypt = self._add_object(encrypt)
        self._encrypt_key = key

    def write(self, stream):
        """Writes the collection of pages added to this object out as a PDF file.

        Stability: Added in v1.0, will exist for all v1.x releases.

        stream - An object to write the file to.  The object must support
                 the write method, and the tell method, similar to a file object."""
        external_reference_map = self._build_external_reference_map()
        self._stack = []
        self._sweep_indirect_references(external_reference_map, self._root)
        self._stack = []

        # Begin writing:
        object_positions = self._write_objects_to(stream)
        xref_location = self._write_cross_reference_table(stream, object_positions)
        self._write_trailer_to(stream)
        stream.write(utils.s2b('\nstartxref\n%s\n%%%%EOF\n' % xref_location))

    def _write_cross_reference_table(self, stream, object_positions):
        xref_location = stream.tell()
        stream.write(b'xref\n')
        stream.write(utils.s2b("0 %s\n" % (len(self._objects) + 1)))
        stream.write(utils.s2b("%010d %05d f \n" % (0, 65535)))
        for offset in object_positions:
            stream.write(utils.s2b("%010d %05d n \n" % (offset, 0)))
        return xref_location

    def _write_objects_to(self, stream):
        object_positions = []
        stream.write(b'%PDF-1.3\n')
        for i in range(len(self._objects)):
            idnum = (i + 1)
            obj = self._objects[i]
            object_positions.append(stream.tell())
            stream.write(utils.s2b(str(idnum) + " 0 obj\n"))
            key = None
            if (self._encrypt is not None) and (not (idnum == self._encrypt.idnum)):
                pack1 = struct.pack("<i", i + 1)[:3]
                pack2 = struct.pack("<i", 0)[:2]
                key = _encrypt(self._encrypt_key, pack1, pack2)
            if is_plain_object(obj):
                obj.write_to_stream(stream)
            else:
                obj.write_to_stream(stream, key)
            stream.write(b'\nendobj\n')
        return object_positions

    def _add_object(self, obj):
        self._objects.append(obj)
        return IndirectObject(len(self._objects), 0, self)

    def _add_page(self, page, callback_add):
        """Common method for inserting or adding a page to this PDF file.

        page - The page to add to the document.  This argument should be
                    an instance of {@link #PageObject PageObject}.
        callback_add - The function which will insert the page in the dictionnary.
                      Takes: page list, page to add."""
        assert page[b'/Type'] == b'/Page'
        page[NameObject(b'/Parent')] = self._pages
        page = self._add_object(page)
        pages = self._pages.get_object()
        callback_add(pages[b'/Kids'], page)
        pages[NameObject(b'/Count')] = NumberObject(pages[b'/Count'] + 1)

    def _build_external_reference_map(self):
        """PDF objects sometimes have circular references to their /Page objects
        inside their object tree (for example, annotations).  Those will be
        indirect references to objects that we've recreated in this PDF.  To
        address this problem, PageObject's store their original object
        reference number, and we add it to the external reference map before
        we sweep for indirect references.  This forces self-page-referencing
        trees to reference the correct new object location, rather than
        copying in a new copy of the page object."""
        external_reference_map = {}
        for ido_index in range(len(self._objects)):
            ido = self._objects[ido_index]
            if isinstance(ido, PageObject) and ido.indirect_ref is not None:
                utils.debug(type(ido), ido.indirect_ref)
                data = ido.indirect_ref
                if data.pdf not in external_reference_map:
                    external_reference_map[data.pdf] = {}
                if data.generation not in external_reference_map[data.pdf]:
                    external_reference_map[data.pdf][data.generation] = {}
                external_reference_map[data.pdf][data.generation][data.idnum] = IndirectObject(ido_index + 1, 0, self)
        return external_reference_map

    def _write_trailer_to(self, stream):
        stream.write(b'trailer\n')
        trailer = DictionaryObject()
        trailer.update({
            NameObject(b'/Size'): NumberObject(len(self._objects) + 1),
            NameObject(b'/Root'): self._root,
            NameObject(b'/Info'): self._info,
        })
        if self._id is not None:
            trailer[NameObject(b'/ID')] = self._id
        if self._encrypt is not None:
            trailer[NameObject(b'/Encrypt')] = self._encrypt
        trailer.write_to_stream(stream)

    def _sweep_indirect_references(self, extern_map, data):
        if isinstance(data, DictionaryObject):
            for key, value in data.items():
                _origvalue = value
                value = self._sweep_indirect_references(extern_map, value)
                if isinstance(value, StreamObject):
                    # a dictionary value is a stream.  streams must be indirect
                    # objects, so we need to change this value.
                    value = self._add_object(value)
                data[key] = value
            return data
        elif isinstance(data, ArrayObject):
            for i in range(len(data)):
                value = self._sweep_indirect_references(extern_map, data[i])
                if isinstance(value, StreamObject):
                    # an array value is a stream.  streams must be indirect
                    # objects, so we need to change this value
                    value = self._add_object(value)
                data[i] = value
            return data
        elif isinstance(data, IndirectObject):
            # internal indirect references are fine
            if data.pdf == self:
                if data.idnum in self._stack:
                    return data
                else:
                    self._stack.append(data.idnum)
                    realdata = self.get_object(data)
                    self._sweep_indirect_references(extern_map, realdata)
                    self._stack.pop()
                    return data
            else:
                newobj = extern_map.get(data.pdf, {}).get(data.generation, {}).get(data.idnum, None)
                if newobj is None:
                    newobj = data.pdf.get_object(data)
                    self._objects.append(None)  # placeholder
                    idnum = len(self._objects)
                    newobj_ido = IndirectObject(idnum, 0, self)
                    if data.pdf not in extern_map:
                        extern_map[data.pdf] = {}
                    if data.generation not in extern_map[data.pdf]:
                        extern_map[data.pdf][data.generation] = {}
                    extern_map[data.pdf][data.generation][data.idnum] = newobj_ido
                    newobj = self._sweep_indirect_references(extern_map, newobj)
                    self._objects[idnum - 1] = newobj
                    return newobj_ido
                return newobj
        else:
            return data


def convert_to_int(d, size):
    if size > 8:
        raise utils.PdfReadError("invalid size in convertToInt")
    d = "\x00\x00\x00\x00\x00\x00\x00\x00" + d
    d = d[-8:]
    return struct.unpack(">q", d)[0]


# ref: pdf1.8 spec section 3.5.2 algorithm 3.2
_encryption_padding = '\x28\xbf\x4e\x5e\x4e\x75\x8a\x41\x64\x00\x4e\x56' + \
                      '\xff\xfa\x01\x08\x2e\x2e\x00\xb6\xd0\x68\x3e\x80\x2f\x0c' + \
                      '\xa9\xfe\x64\x53\x69\x7a'


# Implementation of algorithm 3.2 of the PDF standard security handler,
# section 3.5.2 of the PDF 1.6 reference.
def _alg32(password, rev, keylen, owner_entry, p_entry, id1_entry, metadata_encrypt=True):
    # 1. Pad or truncate the password string to exactly 32 bytes.  If the
    # password string is more than 32 bytes long, use only its first 32 bytes;
    # if it is less than 32 bytes long, pad it by appending the required number
    # of additional bytes from the beginning of the padding string
    # (_encryption_padding).
    password = (password + _encryption_padding)[:32]
    # 2. Initialize the MD5 hash function and pass the result of step 1 as
    # input to this function.
    m = md5(password)
    # 3. Pass the value of the encryption dictionary's /O entry to the MD5 hash
    # function.
    m.update(owner_entry)
    # 4. Treat the value of the /P entry as an unsigned 4-byte integer and pass
    # these bytes to the MD5 hash function, low-order byte first.
    p_entry = struct.pack('<i', p_entry)
    m.update(p_entry)
    # 5. Pass the first element of the file's file identifier array to the MD5
    # hash function.
    m.update(id1_entry)
    # 6. (Revision 3 or greater) If document metadata is not being encrypted,
    # pass 4 bytes with the value 0xFFFFFFFF to the MD5 hash function.
    if rev >= 3 and not metadata_encrypt:
        m.update(b'\xff\xff\xff\xff')
    # 7. Finish the hash.
    md5_hash = m.digest()
    # 8. (Revision 3 or greater) Do the following 50 times: Take the output
    # from the previous MD5 hash and pass the first n bytes of the output as
    # input into a new MD5 hash, where n is the number of bytes of the
    # encryption key as defined by the value of the encryption dictionary's
    # /Length entry.
    if rev >= 3:
        for i in range(50):
            md5_hash = md5(md5_hash[:keylen]).digest()
    # 9. Set the encryption key to the first n bytes of the output from the
    # final MD5 hash, where n is always 5 for revision 2 but, for revision 3 or
    # greater, depends on the value of the encryption dictionary's /Length
    # entry.
    return md5_hash[:keylen]


# Implementation of algorithm 3.3 of the PDF standard security handler,
# section 3.5.2 of the PDF 1.6 reference.
def _alg33(owner_pwd, user_pwd, rev, keylen):
    # steps 1 - 4
    key = _alg33_1(owner_pwd, rev, keylen)
    # 5. Pad or truncate the user password string as described in step 1 of
    # algorithm 3.2.
    user_pwd = (user_pwd + _encryption_padding)[:32]
    # 6. Encrypt the result of step 5, using an RC4 encryption function with
    # the encryption key obtained in step 4.
    val = utils.rc4_encrypt(key, user_pwd)
    # 7. (Revision 3 or greater) Do the following 19 times: Take the output
    # from the previous invocation of the RC4 function and pass it as input to
    # a new invocation of the function; use an encryption key generated by
    # taking each byte of the encryption key obtained in step 4 and performing
    # an XOR operation between that byte and the single-byte value of the
    # iteration counter (from 1 to 19).
    if rev >= 3:
        for i in range(1, 20):
            new_key = ''
            for j in range(len(key)):
                new_key += chr(ord(key[j:j + 1]) ^ i)
            val = utils.rc4_encrypt(new_key, val)
    # 8. Store the output from the final invocation of the RC4 as the value of
    # the /O entry in the encryption dictionary.
    return val


# Steps 1-4 of algorithm 3.3
def _alg33_1(password, rev, keylen):
    # 1. Pad or truncate the owner password string as described in step 1 of
    # algorithm 3.2.  If there is no owner password, use the user password
    # instead.
    password = (password + _encryption_padding)[:32]
    # 2. Initialize the MD5 hash function and pass the result of step 1 as
    # input to this function.
    m = md5(password)
    # 3. (Revision 3 or greater) Do the following 50 times: Take the output
    # from the previous MD5 hash and pass it as input into a new MD5 hash.
    md5_hash = m.digest()
    if rev >= 3:
        for i in range(50):
            md5_hash = md5(md5_hash).digest()
    # 4. Create an RC4 encryption key using the first n bytes of the output
    # from the final MD5 hash, where n is always 5 for revision 2 but, for
    # revision 3 or greater, depends on the value of the encryption
    # dictionary's /Length entry.
    key = md5_hash[:keylen]
    return key


# Implementation of algorithm 3.4 of the PDF standard security handler,
# section 3.5.2 of the PDF 1.6 reference.
def _alg34(password, owner_entry, p_entry, id1_entry):
    # 1. Create an encryption key based on the user password string, as
    # described in algorithm 3.2.
    key = _alg32(password, 2, 5, owner_entry, p_entry, id1_entry)
    # 2. Encrypt the 32-byte padding string shown in step 1 of algorithm 3.2,
    # using an RC4 encryption function with the encryption key from the
    # preceding step.
    u = utils.rc4_encrypt(key, _encryption_padding)
    # 3. Store the result of step 2 as the value of the /U entry in the
    # encryption dictionary.
    return u, key


# Implementation of algorithm 3.4 of the PDF standard security handler,
# section 3.5.2 of the PDF 1.6 reference.
def _alg35(password, rev, keylen, owner_entry, p_entry, id1_entry, _metadata_encrypt):
    # 1. Create an encryption key based on the user password string, as
    # described in Algorithm 3.2.
    key = _alg32(password, rev, keylen, owner_entry, p_entry, id1_entry)
    # 2. Initialize the MD5 hash function and pass the 32-byte padding string
    # shown in step 1 of Algorithm 3.2 as input to this function. 
    m = md5()
    m.update(bytes(_encryption_padding, utils.ENCODING_UTF8))
    # 3. Pass the first element of the file's file identifier array (the value
    # of the ID entry in the document's trailer dictionary; see Table 3.13 on
    # page 73) to the hash function and finish the hash.  (See implementation
    # note 25 in Appendix H.) 
    m.update(id1_entry)
    md5_hash = m.digest()
    # 4. Encrypt the 16-byte result of the hash, using an RC4 encryption
    # function with the encryption key from step 1. 
    val = utils.rc4_encrypt(key, md5_hash)
    # 5. Do the following 19 times: Take the output from the previous
    # invocation of the RC4 function and pass it as input to a new invocation
    # of the function; use an encryption key generated by taking each byte of
    # the original encryption key (obtained in step 2) and performing an XOR
    # operation between that byte and the single-byte value of the iteration
    # counter (from 1 to 19). 
    for _i in range(1, 20):
        new_key = ''
        for j in range(len(key)):
            new_key += chr(ord(key[j:j + 1]) ^ _i)
        val = utils.rc4_encrypt(new_key, val)
    # 6. Append 16 bytes of arbitrary padding to the output from the final
    # invocation of the RC4 function and store the 32-byte result as the value
    # of the U entry in the encryption dictionary. 
    # (implementator note: I don't know what "arbitrary padding" is supposed to
    # mean, so I have used null bytes.  This seems to match a few other
    # people's implementations)
    return val + ('\x00' * 16), key


def _encrypt(encrypt_key, pack1, pack2):
    key = encrypt_key + pack1 + pack2
    assert len(key) == (len(encrypt_key) + 5)
    md5_hash = md5(key).digest()
    key = md5_hash[:min(16, len(encrypt_key) + 5)]
    return key


def _parse_standard_cross_reference_table(stream, pdf_reader):
    # standard cross-reference table
    ref = stream.read(4)
    if not ref[:3] == b'ref':
        raise utils.PdfReadError("xref table read error")
    seek_token(stream)
    while 1:
        num = read_object(stream, pdf_reader)
        seek_token(stream)
        size = read_object(stream, pdf_reader)
        seek_token(stream)
        cnt = 0
        utils.debug(num, size)
        while cnt < size:
            line = stream.read(20)
            # It's very clear in section 3.4.3 of the PDF spec
            # that all cross-reference table lines are a fixed
            # 20 bytes.  However... some malformed PDF files
            # use a single character EOL without a preceeding
            # space.  Detect that case, and seek the stream
            # back one character.  (0-9 means we've bled into
            # the next xref entry, t means we've bled into the
            # text "trailer"):
            if line[-1] in b'0123456789t':
                stream.seek(-1, io.SEEK_CUR)
            offset, generation = line[:16].split(b' ')
            offset, generation = int(offset), int(generation)
            if generation not in pdf_reader.xref:
                pdf_reader.xref[generation] = {}
            if num in pdf_reader.xref[generation]:
                # It really seems like we should allow the last
                # xref table in the file to override previous
                # ones. Since we read the file backwards, assume
                # any existing key is already set correctly.
                pass
            else:
                pdf_reader.xref[generation][num] = offset
            cnt += 1
            num += 1
        seek_token(stream)
        trailertag = stream.read(7)
        if not trailertag == b'trailer':
            # more xrefs!
            stream.seek(-7, io.SEEK_CUR)
        else:
            break
    seek_token(stream)
    new_trailer = read_object(stream, pdf_reader)
    for key, value in new_trailer.items():
        if key not in pdf_reader.trailer:
            pdf_reader.trailer[key] = value
    if b'/Prev' in new_trailer:
        return new_trailer[b'/Prev']
    else:
        return None
