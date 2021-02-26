from PyPDF.pdf import PdfFileReader, PdfFileWriter

if __name__ == "__main__":
    with open("test_input_a.pdf", "rb") as file_a:
        with open("test_input_b.pdf", "rb") as file_b:
            input1 = PdfFileReader(file_a)
            input2 = PdfFileReader(file_b)

            page1 = input1.get_page(0)
            page2 = input2.get_page(0)
            page3 = input2.get_page(1)

            page1.mergePage(page2)
            page1.mergePage(page3)

            output = PdfFileWriter()
            output.add_page(page1)
            with open("test_output.PyPDF", "wb") as file:
                output.write(file)
