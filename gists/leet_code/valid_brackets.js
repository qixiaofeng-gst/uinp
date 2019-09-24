const local = true

/**
 * @param {string} s
 * @return {boolean}
 */
const checkValidString = function(s) {
  const left = '('
  const right = ')'
  const wild = '*'
  const regex_simplifier = /\(\)/g
  while (s.search(regex_simplifier) >= 0) {
    s = s.replace(regex_simplifier, '')
  }
  
  let r = ''
  for (let i = s.length; i > 0; --i) {
    const c = s.charAt(i - 1)
    r += (left == c ? right : (right == c ? left : wild))
  }
  
  if (local) {
    console.log(s)
    console.log(r)
  }
  
  const check = str => {
    let
    required_right = 0,
    wild_count = 0
    for (const c of str) {
      switch (c) {
      case left:
        ++required_right
        break
      case right:
        --required_right
        if (required_right < 0) {
          if (0 == wild_count) {
            return false
          }
          --wild_count
          required_right = 0
        }
        break
      case wild:
        ++wild_count
        break
      }
    }
    if (local) {
      console.log(required_right, wild_count)
    }
    return required_right <= wild_count
  }
  return check(s) && check(r)
}

console.log(checkValidString("(())((())()()(*)(*()(())())())()()((()())((()))(*"))
console.log(checkValidString("(*))"))