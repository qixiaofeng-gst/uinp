const {
  prepare,
  print,
  swap,
} = require('./prepare_to_sort')
const create_heap = require('./heap')

const to_sort = prepare(1000)

const heap_sort = () => {
  const heap = create_heap(to_sort)
  while (heap.pop_to_end() > 0) {}
}

heap_sort()
print(to_sort)
