const {
  prepare,
  print,
  swap,
} = require('./prepare_to_sort')

const to_sort = prepare(1000)

const heap_sort = () => {
  i_parent = i => Math.floor((i - 1) * .5)
  i_left = i => i * 2 + 1
  i_right = i => i * 2 + 2
  // TODO
}

heap_sort()
print(to_sort)
