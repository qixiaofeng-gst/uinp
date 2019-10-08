/**
https://leetcode.com/problems/regular-expression-matching/

Given an input string (s) and a pattern (p), implement regular expression matching with support for '.' and '*'.

'.' Matches any single character.
'*' Matches zero or more of the preceding element.
The matching should cover the entire input string (not partial).

Note:

s could be empty and contains only lowercase letters a-z.
p could be empty and contains only lowercase letters a-z, and characters like . or *.
*/

/**
 * @param {string} s string to be matched
 * @param {string} p the pattern to be used
 * @return {boolean} if the given string matches the pattern
 */
const isMatch = (s, p) => {
  const empty = ''
  if (empty == p) {
    return empty == s
  }
  /*
  1. char
  2. char + *
  3. dot
  4. dot + *
  */
  
  const type_1 = c => (input => (1 == input.length && c == input))
  
  const type_2 = c => (input => {
    if (0 == input.length) {
      return true
    }
    for (const i of input) {
      const valid = i == c
      if (false === valid) {
        return false
      }
    }
    return true
  })
  
  const type_3 = () => (input => 1 == input.length)
  
  const type_4 = () => (input => true)
  
  const pattern_parts = []
  const compile_pattern = () => {
    const dot = '.'
    const process_single = current => {      
      if (dot == current) {
        pattern_parts.push(type_3())
      } else {
        pattern_parts.push(type_1(current))
      }
    }
    const process_pair = current => {
      if (dot == current) {
        pattern_parts.push(type_4())
      } else {
        pattern_parts.push(type_2(current))
      }
    }
    
    const len = p.length
    for (let i = 0; i < len; ++i) {
      const current = p[i]
      if ((len - 1) == i) {
        process_single(current)
        return
      }
      
      const next = p[i + 1]
      if ('*' == next) {
        process_pair(current)
        ++i;
      } else {
        process_single(current)
      }
    }
  }
  compile_pattern()
  
  const pp_len = pattern_parts.length
  const match = (rest, p_index) => {
    const current_pp = pattern_parts[p_index]
    if ((pp_len - 1) == p_index) {
      return current_pp(rest)
    }
    const len = rest.length
    for (let i = 0; i <= len; ++i) {
      const eaten = rest.substr(0, i)
      if (false == current_pp(eaten)) {
        continue
      }
      const next_rest = rest.substr(i, len)
      if (match(next_rest, p_index + 1)) {
        return true
      }
    }
    return false
  }
  
  return match(s, 0)
}

console.log(isMatch('aa', 'a'), false)
console.log(isMatch('aa', 'a*'), true)
console.log(isMatch('ab', '.*'), true)
console.log(isMatch('aab', 'c*a*b'), true)
console.log(isMatch('mississippi', 'mis*is*p*.'), false)
console.log(isMatch('a', ''), false)
console.log(isMatch('', ''), true)
