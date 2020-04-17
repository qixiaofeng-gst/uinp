#ifndef TABLE_UTILITIES_H
#define TABLE_UTILITIES_H

void
generateTableFile();

void
showUnicodeTable();

bool
isTableFileExist();

void
loadTableFromFile(wchar_t *tableInMemory);

extern const int table_string_length;
#define M_table_logic_size 15

#endif // guard end for TABLE_UTILITIES_H
