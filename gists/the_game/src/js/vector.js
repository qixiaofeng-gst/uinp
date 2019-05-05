const XY = (v1, v2) => {
  const x = v1 || 0
  const y = v2 || 0
  const cal_len = (a, b) => Math.sqrt(a * a + b * b)
  const len = cal_len(x, y)
  
  const sub = ({ x: a, y: b }) => XY(x - a, y - b)
  const add = ({ x: a, y: b }) => XY(x + a, y + b)
  const mul = ({ x: a, y: b }) => XY(x * a, y * b)
  const div = ({ x: a, y: b }) => XY(x / a, y / b)
  const mul_n = n => XY(x * n, y * n)
  const div_n = n => XY(x / n, y / n)
  
  const distance = ({ x: a, y: b }) => cal_len(x - a, y - b)
  const normalize = () => ((len > 0) ? XY(x / len, y / len) : XY())
  
  return ({
    x, y, len,
    sub, add, mul, div,
    mul_n, div_n,
    normalize,
    distance,
  })
}