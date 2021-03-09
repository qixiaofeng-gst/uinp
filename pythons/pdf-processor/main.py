import pdf.character_set

if __name__ == '__main__':
    print('Here we go!')
    with open('test_input_b.pdf', 'rb') as file:
        lines = file.readlines()
        for line in lines:
            print(line)
