const ou = (() => {
  const
  copy = obj => {
    const r = {}
    for (const key in obj) {
      const val = obj[key]
      if (val instanceof Object) {
        r[key] = copy(val)
      } else {
        r[key] = val
      }
    }
    return r
  },
  assign = (obj, val) => {
    for (const key in val) {
      obj[key] = val[key]
    }
  }
  
  return new Proxy({
    copy,
    assign,
  }, {
    set: (_, key, val, __) => false,
  })
})()