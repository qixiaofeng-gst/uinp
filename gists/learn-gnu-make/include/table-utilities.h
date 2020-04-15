#ifndef TABLE_UTILITIES_H
#define TABLE_UTILITIES_H

extern
void
generateTableFile();

extern
void
showUnicodeTable();

extern
bool
isTableFileExist();

extern
void
loadTableFromFile(wchar_t *tableInMemory);

extern const int table_string_length;
extern const int table_logic_size;

#endif // guard end for TABLE_UTILITIES_H
