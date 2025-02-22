import { ExpressPeerServer } from 'peer';
import express from 'express';

const app = express();
const server = app.listen(9000, () => {
  console.log('PeerJS server running on http://localhost:8081');
});

const peerServer = ExpressPeerServer(server, {
  path: '/peerjs',
});

app.use('/peerjs', peerServer);