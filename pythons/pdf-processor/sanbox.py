from PyPDF.reader import PdfFileReader
from PyPDF.writer import PdfFileWriter
import PyPDF.utils as utils

"""
TODO:
1. Create proper size page from image. 

Done:
1. Image pages could be saved now.
2. Compress image back.
"""


def test_merge():
    with open("test_input_a.pdf", "rb") as file_a:
        with open("test_input_b.pdf", "rb") as file_b:
            input1 = PdfFileReader(file_a)  # 134 pages
            input2 = PdfFileReader(file_b)  # 325 pages

            page1 = input1.get_page(1)
            page2 = input2.get_page(100)
            page3 = input2.get_page(101)
            utils.debug(page2.get_contents(), type(page2.get_contents()))
            utils.debug(page3.get_contents(), type(page3.get_contents()))

            page1.merge_page(page2)
            page1.merge_page(page3)

            output = PdfFileWriter()
            output.add_page(page1)
            with open("test_output.pdf", "wb") as file:
                output.write(file)


def test_process():
    # with open("test_input_pm_v1.pdf", "rb") as file_b:  # CCITTFaxDecode
    # with open("test_input_taocp_v1.pdf", "rb") as file_b:  # DCTDecode
    with open("test_input_b.pdf", "rb") as file_b:
        input2 = PdfFileReader(file_b)  # 325 pages
        with open("test_output.pdf", "wb") as file:
            output = PdfFileWriter()
            for i in range(input2.get_pages_count()):
                output.add_page(input2.get_page(i))
            output.write(file)


if __name__ == "__main__":
    # test_merge()
    test_process()
