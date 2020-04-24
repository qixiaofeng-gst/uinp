const {
  prepare,
  print,
  swap,
} = require('./prepare_to_sort')

const to_sort = prepare(1000)

const selection_sort = () => {
  const pop_min = (arr, start_index) => {
    let
    cand_index = start_index
    cand = arr[cand_index]
    for (let i = start_index + 1; i < arr.length; ++i) {
      const curr = arr[i]
      if (curr < cand) {
        cand_index = i
        cand = curr
      }
    }
    return cand_index
  }
  for (let i = 0; i < to_sort.length; ++i) {
    const cand_index = pop_min(to_sort, i)
    if (i == cand_index) {
      continue
    }
    swap(to_sort, i, cand_index)
  }
}

selection_sort()
print(to_sort)