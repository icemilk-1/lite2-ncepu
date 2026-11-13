// Verify venue/year/DOI via Crossref. Usage: node crossref.mjs "paper title" ["another"]
const titles = process.argv.slice(2);

for (const t of titles) {
  const url =
    'https://api.crossref.org/works?rows=2&select=DOI,title,container-title,issued,author,type&query.bibliographic=' +
    encodeURIComponent(t);
  try {
    const r = await fetch(url, {
      headers: { 'User-Agent': 'research-script/1.0 (mailto:noreply@example.com)' },
    });
    const j = await r.json();
    console.log(`\n## QUERY: ${t}`);
    const items = (j.message && j.message.items) || [];
    if (!items.length) console.log('  (no crossref match)');
    for (const it of items) {
      const title = (it.title && it.title[0]) || '-';
      const ct = (it['container-title'] && it['container-title'][0]) || '-';
      const yr = it.issued && it.issued['date-parts'] && it.issued['date-parts'][0][0];
      const a = (it.author || []).slice(0, 3).map((x) => `${x.given || ''} ${x.family || ''}`.trim()).join(', ');
      console.log(`  - ${it.DOI} | ${title}`);
      console.log(`    venue: ${ct} | year: ${yr} | type: ${it.type} | ${a}`);
    }
  } catch (e) {
    console.log(`\n## QUERY: ${t}\n  ERROR: ${e.message}`);
  }
}