const {
  prepare,
  print,
  swap,
} = require('./prepare_to_sort')

const to_sort = prepare(1000)

const insertion_sort = () => {
  const count = to_sort.length
  for (let i = 1; i < count; ++i) {
    const anchor = to_sort[i]
    const left_neighbor = i - 1
    for (let j = left_neighbor; j >= 0; --j) {
      if (anchor > to_sort[j]) {
        if (j < left_neighbor) {
          to_sort.splice(i, 1)
          to_sort.splice(j + 1, 0, anchor) 
        }
        break
      }
      if (0 == j) {
        to_sort.splice(i, 1)
        to_sort.splice(0, 0, anchor) 
      }
    }
  }
}

insertion_sort()
print(to_sort)
