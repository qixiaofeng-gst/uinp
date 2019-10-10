/**
https://leetcode.com/problems/longest-valid-parentheses/

Given a string containing just the characters '(' and ')', find the length of the longest valid (well-formed) parentheses substring.

Example 1:
Input: "(()"
Output: 2
Explanation: The longest valid parentheses substring is "()"

Example 2:
Input: ")()())"
Output: 4
Explanation: The longest valid parentheses substring is "()()"
*/

/**
 * @param {string} s
 * @return {number}
 */
const longestValidParentheses = s => {
  /*
  char + neighbor_len
  */
  const empty = ''
  const left = '('
  const right = ')'
  const stack = [{
    c: empty,
    l: 0,
  }]
  const is_tail_left = () => {
    return left == stack[stack.length - 1].c
  }
  const push_left = () => {
    stack.push({
      c: left,
      l: 0,
    })
  }
  const push_right = () => {
    if (is_tail_left()) {
      const { l } = stack.pop()
      stack[stack.length - 1].l += (l + 2)
    } else {
      stack.push({
        c: right,
        l: 0,
      })
    }
  }
  
  for (const c of s) {
    if (left == c) {
      push_left()
    } else {
      push_right()
    }
  }
  
  let max = 0
  for (const { l } of stack) {
    if (max < l) {
      max = l
    }
  }
  return max
}

console.log(longestValidParentheses('(()'), 2)
console.log(longestValidParentheses(')()())'), 4)
