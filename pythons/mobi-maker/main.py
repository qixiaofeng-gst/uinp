# 1. [Done] Test kindlegen executable.
#    - TOC.
#    - Chapter.
#    - Line-break.
#    - `zip -r TestKindleBook.epub TestKindleBook`.
#    - `/var/local/app-binaries/kindlegen-2.9/kindlegen TestKindleBook.epub`.
# 2. Read file line by line.
# 3. Parse line with regex.
# 4. Prepare HTML template.
# 5. Write HTML with parse result and template.
# 6. Use kindlegen to make mobi file.
import re

template_start = '<html lang="zh"><head><title>轮回学府</title>' \
                 '<meta http-equiv="content-type" content="text/html; charset=GB2312"></head><body>'
template_end = '</body></html>'

chapter_title_patterns = [
    r'^\#+(第.+章\s?[^\#]+)\#+\s*$',
]


def process_file():
    with open('lhxf.txt', 'r', encoding = 'GB2312') as file:
        lines_limit = 1000000

        processed_lines_count = 0
        chapters_count = 0
        words_count = 0
        line = file.readline()
        toc = [
            '<h1 id="content">目录</h1>',
        ]
        text = []

        while line and (processed_lines_count < lines_limit):
            search_result = re.search(chapter_title_patterns[0], line)
            line_len = len(line)
            if 1 == line_len:
                line = file.readline()
                continue
            if search_result is not None:
                chapters_count += 1
                chapter_name = search_result.group(1)
                chapter_id = 'c{}'.format(chapters_count)
                toc.append('<p><a href="#{}">{}</a></p>'.format(chapter_id, chapter_name))
                text.append('<h1 id="{}">{}</h1>'.format(chapter_id, chapter_name))
                text.append('<p><a href="#content">回目录</a></p>')
            else:
                words_count += line_len
                text.append('<p>{}</p>'.format(line[:-1]))
            processed_lines_count += 1
            line = file.readline()
        print(
            'chapters count:', chapters_count,
            ', lines count:', processed_lines_count,
            ', words count:', words_count,
        )
        return toc, text


def save_to_html(toc, text):
    with open('lhxf.html', 'w+', encoding = 'GB2312') as file:
        file.writelines([template_start])
        file.writelines(toc)
        file.writelines(text)
        file.writelines([template_end])


def make():
    toc, text = process_file()
    save_to_html(toc, text)


if __name__ == '__main__':
    make()
