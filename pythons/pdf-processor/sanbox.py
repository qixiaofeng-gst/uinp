import PyPDF.utils as utils
from PyPDF.generic import NameObject

if __name__ == '__main__':
    utils.debug(NameObject(b'/Root') == b'/Root')
