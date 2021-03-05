import time
import random
import struct as _s
import PyPDF.utils as _u
import PyPDF.compound as _c
import PyPDF.keys as _k
from hashlib import md5 as _md5
from PyPDF.generic import (
    NameObject, NumberObject, IndirectObject, ByteStringObject,
    ArrayObject, DictionaryObject,
    StreamObject,
    create_string_object, is_plain_object,
)


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
            NameObject(_k.TYPE_KEY): NameObject(b'/Pages'),
            NameObject(b'/Count'): NumberObject(0),
            NameObject(_k.KIDS_KEY): ArrayObject(),
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
            NameObject(_k.TYPE_KEY): NameObject(b'/Catalog'),
            NameObject(b'/Pages'): self._pages,
        })
        self._root = self._add_object(root)

    def write(self, stream):
        """Writes the collection of pages added to this object out as a PDF file.

        Stability: Added in v1.0, will exist for all v1.x releases.

        stream - An object to write the file to.  The object must support
                 the write method, and the tell method, similar to a file object."""
        _u.debug(len(self._objects))
        external_reference_map = self._build_external_reference_map()
        _u.debug(len(self._objects))
        self._stack = []
        self._scan_indirect_references(external_reference_map, self._root)
        self._stack = []
        _u.debug(len(self._objects))

        # Begin writing:
        object_positions = self._write_objects_to(stream)
        xref_location = self._write_cross_reference_table(stream, object_positions)
        self._write_trailer_to(stream)
        stream.write(_u.s2b('\nstartxref\n%s\n%%%%EOF\n' % xref_location))

    def add_blank_page(self, width=None, height=None):
        page = _c.create_blank_page(self, width, height)
        self.add_page(page)
        return page

    def insert_blank_page(self, width=None, height=None, index=0):
        if width is None or height is None and self.get_pages_count() > index:
            oldpage = self.get_page(index)
            width = oldpage.media_box.get_width()
            height = oldpage.media_box.get_height()
        page = _c.create_blank_page(self, width, height)
        self.insert_page(page, index)
        return page

    def get_object(self, ido):
        if not ido.parent == self:
            raise ValueError("PyPDF must be self")
        return self._objects[ido.idnum - 1]

    def add_page(self, page):
        self._add_page(page, list.append)

    def insert_page(self, page, index=0):
        self._add_page(page, lambda l, p: l.insert(index, p))

    def get_page(self, page_number):
        """Retrieves a page by number from this PDF file."""
        pages = self._pages.get_object()
        return pages[_k.KIDS_KEY][page_number].get_object()

    def get_pages_count(self):
        pages = self._pages.get_object()
        return int(pages[NameObject(b'/Count')])

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
        o = ByteStringObject(_u.algorithm_33(owner_pwd, user_pwd, rev, keylen))
        id_1 = _md5(bytes(repr(time.time()), _u.ENCODING_UTF8)).digest()
        id_2 = _md5(bytes(repr(random.random()), _u.ENCODING_UTF8)).digest()
        self._id = ArrayObject((ByteStringObject(id_1), ByteStringObject(id_2)))
        if rev == 2:
            u, key = _u.algorithm_34(user_pwd, o, p, id_1)
        else:
            assert rev == 3
            u, key = _u.algorithm_35(user_pwd, rev, keylen, o, p, id_1, False)
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

    def _write_cross_reference_table(self, stream, object_positions):
        xref_location = stream.tell()
        stream.write(b'xref\n')
        stream.write(_u.s2b("0 %s\n" % (len(self._objects) + 1)))
        stream.write(_u.s2b("%010d %05d f \n" % (0, 65535)))
        for offset in object_positions:
            stream.write(_u.s2b("%010d %05d n \n" % (offset, 0)))
        return xref_location

    def _write_objects_to(self, stream):
        object_positions = []
        stream.write(b'%PDF-1.3\n')
        for i in range(len(self._objects)):
            idnum = (i + 1)
            obj = self._objects[i]
            object_positions.append(stream.tell())
            stream.write(_u.s2b(str(idnum) + " 0 obj\n"))
            key = None
            if (self._encrypt is not None) and (not (idnum == self._encrypt.idnum)):
                pack1 = _s.pack("<i", i + 1)[:3]
                pack2 = _s.pack("<i", 0)[:2]
                key = _u.encrypt(self._encrypt_key, pack1, pack2)
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
        assert page[_k.TYPE_KEY] == b'/Page'
        page[NameObject(b'/Parent')] = self._pages
        page = self._add_object(page)
        pages = self._pages.get_object()
        callback_add(pages[_k.KIDS_KEY], page)
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
            if isinstance(ido, _c.PageObject) and ido.indirect_ref is not None:
                _u.debug(type(ido), ido.indirect_ref)
                data = ido.indirect_ref
                if data.parent not in external_reference_map:
                    external_reference_map[data.parent] = {}
                if data.generation not in external_reference_map[data.parent]:
                    external_reference_map[data.parent][data.generation] = {}
                external_reference_map[data.parent][data.generation][data.idnum] = IndirectObject(
                    ido_index + 1, 0, self
                )
        return external_reference_map

    def _scan_indirect_references(self, extern_map, data):
        if isinstance(data, DictionaryObject):
            for key, value in data.items():
                _origvalue = value
                value = self._scan_indirect_references(extern_map, value)
                if isinstance(value, StreamObject):
                    # A dictionary value is a stream.
                    # Streams must be indirect objects, so we need to change this value.
                    value = self._add_object(value)
                data[key] = value
            return data
        elif isinstance(data, ArrayObject):
            for i in range(len(data)):
                value = self._scan_indirect_references(extern_map, data[i])
                if isinstance(value, StreamObject):
                    # an array value is a stream.  streams must be indirect
                    # objects, so we need to change this value
                    value = self._add_object(value)
                data[i] = value
            return data
        elif isinstance(data, IndirectObject):
            # internal indirect references are fine
            if data.parent == self:
                if data.idnum in self._stack:
                    return data
                else:
                    self._stack.append(data.idnum)
                    realdata = self.get_object(data)
                    self._scan_indirect_references(extern_map, realdata)
                    self._stack.pop()
                    return data
            else:
                newobj = extern_map.get(data.parent, {}).get(data.generation, {}).get(data.idnum, None)
                if newobj is None:
                    newobj = data.parent.get_object(data)
                    self._objects.append(None)  # placeholder
                    idnum = len(self._objects)
                    newobj_ido = IndirectObject(idnum, 0, self)
                    if data.parent not in extern_map:
                        extern_map[data.parent] = {}
                    if data.generation not in extern_map[data.parent]:
                        extern_map[data.parent][data.generation] = {}
                    extern_map[data.parent][data.generation][data.idnum] = newobj_ido
                    newobj = self._scan_indirect_references(extern_map, newobj)
                    self._objects[idnum - 1] = newobj
                    return newobj_ido
                return newobj
        else:
            return data

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
