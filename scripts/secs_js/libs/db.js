/** Database Infrastructure. (Based on mongodb) */
const { MongoClient:db, ObjectID:OID } = require('mongodb')

let connect_promise = false

const url = 'mongodb://localhost:27017'

const connect = cb => {
  if (connect_promise) {
    return connect_promise
  }
  connect_promise = new Promise((resolve, reject) => {
    const ts = Date.now()
    db.connect(url, { useNewUrlParser: true }).then(client => {
      console.log(`Connect MongoDB cost ${Date.now() - ts}ms.`)
      resolve(client.db(db_name))
    }).catch(error => {
      console.log('Failed to connect MongoDB. Error:', error)
      reject(error)
    })
  })
  return connect_promise
}

const DB_Table = table_name => {
  const get = query => new Promise((resolve, reject) => connect().then(db_instance => {
    const ts = Date.now()
    db_instance.collection(table_name).findOne(query).then(entry => {
      console.log(`Get operation on ${table_name} cost ${Date.now() - ts}ms.`)
      resolve(entry)
    }).catch(error => {
      console.log(`Failed to get on ${table_name}. Error:`, error)
      reject(error)
    })
  }).catch(error => {
    reject(error)
  }))

  const list = query => new Promise((resolve, reject) => connect().then(db_instance => {
    const ts = Date.now()
    db_instance.collection(table_name).find(query).toArray().then(entries => {
      console.log(`List operation on ${table_name} cost ${Date.now() - ts}ms.`)
      resolve(entries)
    }).catch(error => {
      console.log(`Failed to list on ${table_name}. Error:`, error)
      reject(error)
    })
  }).catch(error => {
    reject(error)
  }))

  const insert = (obj, cb) => new Promise((resolve, reject) => connect().then(db_instance => {
    const ts = Date.now()
    db_instance.collection(table_name).insertOne(obj).then(({ result, ops: [{ _id }] }) => {
      console.log(`Insert operation on ${table_name} cost ${Date.now() - ts}ms.`)
      resolve({ result, _id })
    }).catch(error => {
      console.log(`Failed to insert on ${table_name}. Error:`, error)
      reject(error)
    })
  }).catch(error => {
    reject(error)
  }))

  const update = (filter, obj) => new Promise((resolve, reject) => connect().then(db_instance => {
    const ts = Date.now()
    db_instance.collection(table_name).updateMany(filter, {
      $set: obj
    }).then(({ result }) => {
      console.log(`UpdateWith operation on ${table_name} cost ${Date.now() - ts}ms.`)
      resolve(result)
    }).catch(error => {
      console.log(`Failed to update on ${table_name}. Error:`, error)
      reject(error)
    })
  }).catch(error => {
    reject(error)
  }))

	const exec = () => new Promise((resolve, reject) => connect().then(db_instance => {
		const ts = Date.now()
    const log_ts = () => console.log(`Free querying on ${table_name} cost ${Date.now() - ts}ms.`)
    resolve({
      table: db_instance.collection(table_name),
      log_ts
    })
  }).catch(error => {
    reject(error)
  }))

  return ({
    get,
    list,
    insert,
    update,
    exec
  })
}

const db_name = 'task_server'
module.exports = {
  user: DB_Table('user'),
  task: DB_Table('task')
}
