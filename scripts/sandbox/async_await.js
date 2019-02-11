const one_second = 1000
// smallest number is the innermost stratum

const stratum_0a = () => new Promise((resolve, _) => {
  console.log('[0a] started, expected returned after one second')
  setTimeout(() => {
    console.log(`[0a] to be done`)
    resolve('[0a] done')
  }, one_second)
})

const stratum_0b = () => new Promise((resolve, _) => {
  console.log('[0b] started, expected returned after two second')
  throw new Error('The error in normal block can be catched')
  setTimeout(() => {
    console.log(`[0b] to be done`)
    //resolve('[0b] done')
    throw new Error(`
      The error in a timeout function will terminate whole process
      It is not going to be catched with try...catch
    `)
  }, one_second * 2)
})

const stratum_1a = async () => {
  try {
    console.log('[1a] started')
    const ts = Date.now()
    const a = await stratum_0a()
    console.log(`[1a] got a='${a}' after ${Date.now() - ts}ms`)
    const b = await stratum_0b()
    console.log(`[1a] done, after ${Date.now() - ts}ms, ${a}, ${b}`)
  } catch(err) {
    console.log('[1a] damn it:', err)
  }
}

const stratum_1b = async () => {
  try {
    console.log('[1b] started')
    const ts = Date.now()
    const a = stratum_0a()
    const b = stratum_0b()
    console.log(`[1b] after ${Date.now() - ts}ms, ${a}, ${b}`)
    console.log(`[1b] b: ${await b}, ${Date.now() - ts}ms later`)
    console.log(`[1b] a: ${await a}, ${Date.now() - ts}ms later`)
    console.log('[1b] done')
  } catch (err) {
    console.log('[1b] damn it:', err)
  }
}

const stratum_2 = async () => {
  console.log('[2] started')
  const ts = Date.now()
  await stratum_1a()
  await stratum_1b()
  console.log('[2] done')
}

try {
  stratum_2()/*.then(() => {
    console.log(`[2] whatever after`)
  })*/
} catch(err) {
  console.log('end with:', err)
}
