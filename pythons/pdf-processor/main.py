from pdf.pdf import PdfFileReader, PdfFileWriter

if __name__ == "__main__":
    output = PdfFileWriter()

    with open("test_input_a.pdf", "rb") as file:
        input1 = PdfFileReader(file)

    page1 = input1.get_page(0)

    with open("test_input_b.pdf.pdf", "rb") as file:
        input2 = PdfFileReader(file)

    page2 = input2.get_page(0)
    page3 = input2.get_page(1)

    page1.mergePage(page2)
    page1.mergePage(page3)

    output.add_page(page1)
    with open("test_output.pdf", "wb") as file:
        output.write(file)
