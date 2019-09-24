const {
  swap,
} = require('./prepare_to_sort')

module.exports = arr => {
  let limit = arr.length
  const
  i_parent = i => Math.floor((i - 1) * .5),
  i_left = i => i * 2 + 1,
  i_right = i => i * 2 + 2,
  sift = start => {
    let
    root = start,
    left = 0
    while ((left = i_left(root)) < limit) {
      const right = left + 1
      let to_swap = root
      if (arr[to_swap] < arr[left]) {
        to_swap = left
      }
      if (right < limit && arr[to_swap] < arr[right]) {
        to_swap = right
      }
      if (to_swap == root) {
        return
      }
      swap(arr, to_swap, root)
      root = to_swap
    }
  },
  heapify = () => {
    let start = i_parent(limit - 1)
    while (start >= 0) {
      sift(start)
      --start
    }
  },
  pop_to_end = () => {
    swap(arr, 0, --limit)
    sift(0)
    return limit
  },
  
  calc_height = i => {
    let
    trait = i,
    count = 1
    while ((trait = (trait >> 1)) > 0) {
      ++count
    }
    return count
  },
  height = calc_height(limit),
  level = i => height - calc_height(i + 1),
  print = () => {
    const spaces = c => {
      let r = ''
      for (let i = 0; i < c; ++i) {
        r += ' '
      }
      return r
    }
    let
    last_level = level(0),
    output = spaces(last_level * 2)
    for (let i = 0; i < limit; ++i) {
      const curr_level = level(i)
      if (curr_level < last_level) {
        last_level = curr_level
        output += '\n'
      } else if (0 == curr_level) {
        output += ' '
      }
      if (i > 0) {
        output += spaces(last_level * 2)
      }
      output += arr[i]
    }
    console.log(output)
  }
  
  heapify()
  
  return ({
    pop_to_end,
  })
}