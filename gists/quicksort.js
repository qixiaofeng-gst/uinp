const {
  prepare,
  print,
  swap,
} = require('./prepare_to_sort')

const to_sort = prepare(1000)

const quicksort = () => {
  const inner_sort = (start, end) => {
    if (start >= end) {
      return
    }
    
    let pivot = start
    const pivot_value = to_sort[pivot]
    for (let i = start + 1; i <= end; ++i) {
      const current_value = to_sort[i]
      if (pivot_value < current_value) {
        continue
      }
      const neighbor = pivot + 1
      if (i > neighbor) {
        swap(to_sort, i, neighbor)
      }
      swap(to_sort, neighbor, pivot)
      pivot = neighbor
    }
    inner_sort(start, pivot - 1)
    inner_sort(pivot + 1, end)
  }
  inner_sort(0, to_sort.length - 1)
}

quicksort()
print(to_sort)