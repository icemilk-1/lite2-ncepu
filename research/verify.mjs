// Verify papers against arXiv Atom API; print id / venue refs / abstract snippet.
const titles = process.argv.slice(2);
const MAX = Number(process.env.ABS_CHARS || 420);

async function q(title) {
  const url =
    'https://export.arxiv.org/api/query?search_query=ti:%22' +
    encodeURIComponent(title).replace(/%20/g, '+') +
    '%22&max_results=2';
  const r = await fetch(url, { headers: { 'User-Agent': 'research-script/1.0' } });
  const xml = await r.text();
  return [...xml.matchAll(/<entry>([\s\S]*?)<\/entry>/g)].map((m) => {
    const e = m[1];
    const pick = (tag) => {
      const mm = e.match(new RegExp(`<${tag}[^>]*>([\\s\\S]*?)</${tag}>`));
      return mm ? mm[1].replace(/\s+/g, ' ').trim() : '';
    };
    const authors = [...e.matchAll(/<name>([\s\S]*?)<\/name>/g)].map((x) => x[1].trim());
    return {
      id: pick('id').replace('http://arxiv.org/abs/', ''),
      title: pick('title'),
      published: pick('published').slice(0, 10),
      journal_ref: pick('arxiv:journal_ref'),
      doi: pick('arxiv:doi'),
      authors: authors.slice(0, 3).join(', ') + (authors.length > 3 ? ' et al.' : ''),
      summary: pick('summary'),
    };
  });
}

for (const t of titles) {
  console.log(`\n## ${t}`);
  let res;
  try {
    res = await q(t);
  } catch (err) {
    console.log('  ERROR ' + err.message);
    continue;
  }
  if (!res.length) {
    console.log('  (no arxiv match)');
    continue;
  }
  for (const r of res.slice(0, 1)) {
    console.log(`  ${r.id} | ${r.title}`);
    console.log(`  ${r.published} | ${r.authors}`);
    console.log(`  jr: ${r.journal_ref || '-'} | doi: ${r.doi || '-'}`);
    console.log(`  ABS: ${r.summary.slice(0, MAX)}`);
  }
  await new Promise((r) => setTimeout(r, 1200));
}