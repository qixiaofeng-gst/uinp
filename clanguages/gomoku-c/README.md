# Naming Convention
## File Name
Rules:
1. All words are **lower case**.
1. Always connect words with **dash**. No space. No underscore.

Examples:
- example-file.h
- hello-file-1-2-3.c
## Macro Name
Rules:
1. Prefix function like macro with `M_`.
1. Prefix constant like macro with `m_`.
1. All words are **lower case**.
1. Always connect words with **underscore**.

Examples:
- M_make_function
- m_a_constant
## Function Name
Rules:
1. All words are lower case.
1. Always connect words with **underscore**.

Examples:
- try_show_your_name
- test_talent_of
## Local Variable Name
Rules:
1. Camel style.
1. First character is **lower case**.

Examples:
- aLocalVariable
- neverBeSingleWord
- neverUseNumbers
- useCharacterA
- useCharacterB
## Global Constant/Variable Name
Rules:
1. Prefix constant with `G_`.
1. Prefix variable with `g_`.
1. All words are **lower case**.
1. Always connect words with **underscore**.

Examples:
- G_a_constant
- g_a_variable
## Structure Name
Rules:
1. Camel style.
1. First character is **upper case**.

Examples:
- BasicLinkNode.
- TreeRootNode.
## Union Name
Rules:
1. Prefix structure name with `U_`.

Examples:
- U_BasicUnion.
- U_HouseBase.
## Enum Name
Rules:
1. Prefix structure name with `E_`.
1. Members use function name rules with prefix `e_`.

Examples:
- E_Month, E_WeekDay
- `enum {e_signed_integer, e_unsigned_long} E_NumberType;`

## Usefule prefixes and suffixes
1. First character is `_` means private.   
Things that are only available in current file scope are private.
1. `cb_` means callback function.
1. `ptr_` means pointer.
1. `arr_` means array.
1. `_t` means type.
