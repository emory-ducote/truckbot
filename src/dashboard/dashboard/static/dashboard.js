const POLL_MS = 500;
const STALE_AFTER = 2.0; // seconds without /health/system before the page shows stale

function tile(name, state, detail, stamp) {
  const div = document.createElement('div');
  div.className = 'tile ' + state;
  const nameDiv = document.createElement('div');
  nameDiv.className = 'name';
  nameDiv.textContent = name + ' - ' + state;
  const detailDiv = document.createElement('div');
  detailDiv.className = 'detail';
  detailDiv.textContent = detail;
  const stampDiv = document.createElement('div');
  stampDiv.className = 'stamp';
  stampDiv.textContent = 'timestamp ' + new Date(stamp * 1000).toLocaleTimeString();
  div.append(nameDiv, detailDiv, stampDiv);
  return div;
}

function setOverall(state, text) {
  const overall = document.getElementById('overall');
  overall.className = state;
  overall.textContent = text;
}

async function refresh() {
  try {
    const status = await (await fetch('/status')).json();
    if (status.age === null) {
      setOverall('STALE', 'No data from /health/system');
    } else if (status.age > STALE_AFTER) {
      setOverall('STALE', 'No data for ' + status.age.toFixed(1) + ' s');
    } else {
      setOverall(status.state, 'System ' + status.state);
    }
    document.getElementById('tiles').replaceChildren(
      ...status.modules.map(m => tile(m.name, m.state, m.detail, m.stamp)));
  } catch (e) {
    setOverall('STALE', 'Dashboard unreachable');
  }
}

refresh();
setInterval(refresh, POLL_MS);
