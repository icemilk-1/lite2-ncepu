// Journal-scoped Crossref discovery. Usage: node discover.mjs "<issn>" "<query>" <fromYear> [rows]
const [issn, query, fromYear = '2024', rows = '25'] = process.argv.slice(2);

const url =
  `https://api.crossref.org/journals/${issn}/works?rows=${rows}` +
  `&select=DOI,title,container-title,issued,author` +
  `&filter=from-pub-date:${fromYear}-01-01` +
  `&sort=published&order=desc` +
  `&query.bibliographic=${encodeURIComponent(query)}`;

async function get(u, tries = 4) {
  for (let i = 0; i < tries; i++) {
    try {
      const r = await fetch(u, {
        headers: { 'User-Agent': 'research-script/1.0 (mailto:noreply@example.com)' },
      });
      const txt = await r.text();
      if (!txt.trim()) throw new Error('empty body');
      return JSON.parse(txt);
    } catch (e) {
      if (i === tries - 1) throw e;
      await new Promise((res) => setTimeout(res, 1500 * (i + 1)));
    }
  }
}

const j = await get(url);
const items = (j.message && j.message.items) || [];
console.log(`## ISSN ${issn} | query="${query}" | from ${fromYear} | ${items.length} hits`);
for (const it of items) {
  const title = (it.title && it.title[0]) || '-';
  const yr = it.issued && it.issued['date-parts'] && it.issued['date-parts'][0][0];
  const a = (it.author || []).slice(0, 2).map((x) => `${x.given || ''} ${x.family || ''}`.trim()).join(', ');
  console.log(`- ${yr} | ${it.DOI} | ${title} | ${a}`);
}