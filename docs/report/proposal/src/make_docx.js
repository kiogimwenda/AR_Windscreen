// make_docx.js — builds a Word version of a proposal edition from html_to_blocks.py's JSON.
//
//   python3 src/html_to_blocks.py <edition>.html blocks.json
//   node src/make_docx.js blocks.json figures/jkuat_logo.png out.docx   (needs `npm install docx`)
//
// Layout follows the HTML/PDF edition: the concept paper's cover (A4, logo, bold underlined
// centred lines, bordered student table), then front matter and body with centred page numbers.
// Times New Roman stands in for Liberation Serif (the same metrics; installed with Word).
// The table of contents is a real Word field: open the file in Word and it is filled in
// (References > Update Table, or F9).

const fs = require("fs");
const {
  Document, Packer, Paragraph, TextRun, ImageRun, Table, TableRow, TableCell, WidthType,
  AlignmentType, BorderStyle, HeadingLevel, TableOfContents, Footer, PageNumber, LevelFormat,
  TabStopType, PageBreak, TableLayoutType, VerticalAlign,
} = require("docx");

const [, , blocksPath, logoPath, outPath] = process.argv;
const blocks = JSON.parse(fs.readFileSync(blocksPath, "utf8"));
const FONT = "Times New Roman";
const BODY = 23;          // half-points: 11.5 pt
const TEXT_W = 11906 - 2 * 1440;  // A4 width minus 1" margins, in DXA

const none = { style: BorderStyle.NONE, size: 0, color: "FFFFFF" };
const rule = (sz) => ({ style: BorderStyle.SINGLE, size: sz, color: "000000" });
const thin = { style: BorderStyle.SINGLE, size: 2, color: "999999" };

function runs(rs, extra = {}) {
  return rs.map((r) => new TextRun({
    text: r.t, bold: r.b || extra.bold, italics: r.i || extra.italics,
    subScript: r.sub, superScript: r.sup, font: FONT, size: extra.size || BODY,
  }));
}

// --- Cover (its own section, no page number) ------------------------------------------------
function cover(b) {
  const out = [];
  out.push(new Paragraph({
    alignment: AlignmentType.CENTER, spacing: { before: 240, after: 360 },
    children: [new ImageRun({ type: "png", data: fs.readFileSync(logoPath),
                              transformation: { width: 187, height: 187 } })],
  }));
  const line = (l, gapBefore = 0) => new Paragraph({
    alignment: AlignmentType.CENTER, spacing: { before: gapBefore, after: 0, line: 276 },
    children: [new TextRun({ text: l.text, bold: true, underline: l.u ? {} : undefined,
                             font: FONT, size: 28 })],
  });
  const L = b.lines;
  // Groups as on the concept paper: institution (3), proposal, student details, table, course
  // block (4), lecturer and date.
  out.push(line(L[0]), line(L[1]), line(L[2]));
  out.push(line(L[3], 280));
  out.push(line(L[4], 280));
  const cellW = 3800;
  const cell = (t, head) => new TableCell({
    width: { size: cellW, type: WidthType.DXA }, verticalAlign: VerticalAlign.CENTER,
    margins: { top: 80, bottom: 80, left: 120, right: 120 },
    children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [
      new TextRun({ text: t, bold: true, underline: head ? {} : undefined, font: FONT, size: 26 })] })],
  });
  out.push(new Paragraph({ spacing: { before: 120, after: 0 }, children: [] }));
  out.push(new Table({
    alignment: AlignmentType.CENTER, columnWidths: [cellW, cellW],
    width: { size: 2 * cellW, type: WidthType.DXA }, layout: TableLayoutType.FIXED,
    rows: b.student.map((r, i) => new TableRow({ children: r.map((t) => cell(t, i === 0)) })),
  }));
  out.push(line(L[5], 280), line(L[6]), line(L[7]), line(L[8]));
  out.push(line(L[9], 280), line(L[10]));
  return out;
}

// --- Tables ----------------------------------------------------------------------------------
function table(b) {
  const ncol = b.rows[0].length;
  const widths = ncol === 2 ? [1700, 5300]
                            : [1900, 2150, 2400, TEXT_W - 1900 - 2150 - 2400];
  const total = widths.reduce((a, c) => a + c, 0);
  const size = ncol === 2 ? 21 : 19;
  const last = b.rows.length - 1;
  return new Table({
    alignment: AlignmentType.CENTER, columnWidths: widths, layout: TableLayoutType.FIXED,
    width: { size: total, type: WidthType.DXA },
    borders: { top: none, bottom: none, left: none, right: none,
               insideHorizontal: none, insideVertical: none },
    rows: b.rows.map((row, ri) => new TableRow({
      tableHeader: ri === 0, cantSplit: true,
      children: row.map((c, ci) => new TableCell({
        width: { size: widths[ci], type: WidthType.DXA },
        margins: { top: ri === 0 || b.lined ? 50 : 10, bottom: ri === 0 || b.lined ? 50 : 10,
                   left: 100, right: 100 },
        borders: {
          top: ri === 0 ? rule(12) : ri === 1 ? rule(6) : b.lined ? thin : none,
          bottom: ri === last ? rule(12) : ri === 0 ? rule(6) : none,
          left: none, right: none,
        },
        children: [new Paragraph({ spacing: { before: 0, after: 0, line: 252 },
                                   children: runs(c.runs, { size }) })],
      })),
    })),
  });
}

// --- References ------------------------------------------------------------------------------
function refs(b) {
  return b.items.map((rs) => {
    const first = rs[0].t.match(/^\[(\d+)\]\s*(.*)$/);
    const rest = first ? [{ ...rs[0], t: first[2] }, ...rs.slice(1)] : rs;
    return new Paragraph({
      alignment: AlignmentType.LEFT, spacing: { after: 90, line: 252 },
      indent: { left: 620, hanging: 620 }, tabStops: [{ type: TabStopType.LEFT, position: 620 }],
      children: [new TextRun({ text: first ? `[${first[1]}]\t` : "", font: FONT, size: 20 }),
                 ...runs(rest, { size: 20 })],
    });
  });
}

// --- Body --------------------------------------------------------------------------------------
const heading = (text, level, pageBreak) => new Paragraph({
  heading: level, pageBreakBefore: pageBreak, keepNext: true,
  children: [new TextRun({ text, font: FONT, bold: true, color: "000000",
                           size: level === HeadingLevel.HEADING_1 ? 32
                               : level === HeadingLevel.HEADING_2 ? 26 : 24 })],
});
// Front-matter titles look like headings but stay out of the table of contents.
const frontTitle = (text, pageBreak) => new Paragraph({
  pageBreakBefore: pageBreak, keepNext: true, spacing: { after: 240 },
  children: [new TextRun({ text, font: FONT, bold: true, size: 32 })],
});

const coverBlocks = [];
const numberingRefs = [];  // one decimal list definition per ordered list, so each restarts at 1
const body = [];
let breakNext = false;
let first = true;
for (const b of blocks) {
  switch (b.type) {
    case "cover": coverBlocks.push(...cover(b)); break;
    case "pagebreak": breakNext = !first; break;
    case "h1":
      if (b.text === "Abbreviations and Acronyms" || b.text === "Table of Contents")
        body.push(frontTitle(b.text, breakNext));
      else body.push(heading(b.text, HeadingLevel.HEADING_1, breakNext));
      breakNext = false; first = false; break;
    case "h2": body.push(heading(b.text, HeadingLevel.HEADING_2, false)); break;
    case "h3": body.push(heading(b.text, HeadingLevel.HEADING_3, false)); break;
    case "p": body.push(new Paragraph({ alignment: AlignmentType.JUSTIFIED, spacing: { after: 140 },
                                        children: runs(b.runs) })); break;
    case "list": {
      const reference = b.ordered ? `num${numberingRefs.length}` : "bullets";
      if (b.ordered) numberingRefs.push(reference);
      b.items.forEach((it) => body.push(new Paragraph({
        alignment: AlignmentType.JUSTIFIED, spacing: { after: 80 },
        numbering: { reference, level: 0 }, children: runs(it),
      })));
      break;
    }
    case "tabcap": body.push(new Paragraph({ alignment: AlignmentType.CENTER, keepNext: true,
      spacing: { before: 200, after: 80 },
      children: [new TextRun({ text: b.text, italics: true, font: FONT, size: 20 })] })); break;
    case "table":
      body.push(table(b));
      body.push(new Paragraph({ spacing: { after: 120 }, children: [] }));
      break;
    case "toc":
      body.push(new TableOfContents("Table of Contents", { hyperlink: true, headingStyleRange: "1-2" }));
      break;
    case "refs": body.push(...refs(b)); break;
    default: break;
  }
}

const numbering = {
  config: [
    { reference: "bullets", levels: [{ level: 0, format: LevelFormat.BULLET, text: "•",
      alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 540, hanging: 300 } } } }] },
    ...numberingRefs.map((reference) => ({ reference, levels: [{ level: 0, format: LevelFormat.DECIMAL,
      text: "%1.", alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 540, hanging: 360 } } } }] })),
  ],
};

const pageNo = new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER,
  children: [new TextRun({ children: [PageNumber.CURRENT], font: FONT, size: 22 })] })] });
const page = { size: { width: 11906, height: 16838 },
               margin: { top: 1440, bottom: 1300, left: 1440, right: 1440 } };

const doc = new Document({
  creator: "Kiogora Ian Mwenda",
  title: "Virtualized AR Windscreen Display with Semi-Autonomous Driving Support System: Project Proposal",
  description: "Final year project proposal: introduction and literature review",
  styles: {
    default: { document: { run: { font: FONT, size: BODY },
                           paragraph: { spacing: { line: 300 } } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: FONT, size: 32, bold: true, color: "000000" },
        paragraph: { spacing: { before: 0, after: 240 }, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: FONT, size: 26, bold: true, color: "000000" },
        paragraph: { spacing: { before: 300, after: 120 }, outlineLevel: 1 } },
      { id: "Heading3", name: "Heading 3", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: FONT, size: 24, bold: true, color: "000000" },
        paragraph: { spacing: { before: 220, after: 100 }, outlineLevel: 2 } },
    ],
  },
  numbering,
  sections: [
    { properties: { page }, children: coverBlocks },
    { properties: { page: { ...page, pageNumbers: { start: 1 } } },
      footers: { default: pageNo }, children: body },
  ],
});

Packer.toBuffer(doc).then((buf) => {
  fs.writeFileSync(outPath, buf);
  console.log(`wrote ${outPath} (${(buf.length / 1024).toFixed(0)} KB)`);
});
