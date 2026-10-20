#!/usr/bin/env python3
"""Export the Vietnamese Markdown report to a readable PDF with embedded fonts.

Install python3-reportlab, then run this script. The PDF remains a draft until
the student fills class information and the public submission links.
"""
import argparse
import re
from pathlib import Path
from xml.sax.saxutils import escape
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, KeepTogether


ROOT = Path(__file__).resolve().parents[1]
pdfmetrics.registerFont(TTFont('DejaVu', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('DejaVuBold', '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'))
pdfmetrics.registerFontFamily('DejaVu', normal='DejaVu', bold='DejaVuBold', italic='DejaVu', boldItalic='DejaVuBold')
styles = getSampleStyleSheet()
for key in styles.byName:
    styles[key].fontName = 'DejaVu'
styles['Normal'].fontSize = 9
styles['Normal'].leading = 13
styles['Normal'].spaceAfter = 5
styles['Title'].fontName = 'DejaVuBold'
styles['Title'].fontSize = 16
styles['Title'].leading = 21
styles['Heading2'].fontName = 'DejaVuBold'
styles['Heading2'].fontSize = 11
styles['Heading2'].leading = 16
small = ParagraphStyle('TableCell', parent=styles['Normal'], fontSize=7.5, leading=10, spaceAfter=0)


def inline(text):
    text = escape(text)
    text = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', text)
    text = re.sub(r'`([^`]+)`', r'<font color="#215080">\1</font>', text)
    text = re.sub(r'\[([^]]+)\]\((https?://[^)]+)\)', r'<link href="\2" color="#215080">\1</link>', text)
    text = re.sub(r'\[([^]]+)\]\(([^)]+)\)', r'\1', text)
    return text


def parse(path):
    lines=path.read_text().splitlines(); output=[]; paragraph=[]; i=0
    def flush():
        if paragraph:
            output.append(Paragraph(inline(' '.join(paragraph)), styles['Normal']));paragraph.clear()
    while i < len(lines):
        line=lines[i]
        match = re.fullmatch(r'!\[([^]]*)\]\(([^)]+)\)', line.strip())
        if match:
            flush()
            image_path = (path.parent / match[2]).resolve()
            if image_path.is_file():
                from PIL import Image as PILImage
                with PILImage.open(image_path) as picture:
                    w, h = picture.size
                scale = min(440 / w, 255 / h)
                output.append(KeepTogether([
                    Image(str(image_path), width=w * scale, height=h * scale),
                    Paragraph(inline(match[1]), small), Spacer(1, 8)]))
            else:
                output.append(Paragraph(inline('Chưa có ảnh thực tế: ' + match[1]), small))
            i += 1
            continue
        if line.startswith('|'):
            flush(); rows=[]
            while i < len(lines) and lines[i].startswith('|'):
                cells=[c.strip() for c in lines[i].strip('|').split('|')]
                if not all(re.fullmatch(r'[:\- ]+',c) for c in cells):
                    rows.append([Paragraph(inline(c),small) for c in cells])
                i+=1
            table=Table(rows,repeatRows=1,hAlign='LEFT',colWidths=[(A4[0]-80)/len(rows[0])]*len(rows[0]))
            table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#E8EDF3')),
                ('GRID',(0,0),(-1,-1),.3,colors.HexColor('#BBBBBB')),('VALIGN',(0,0),(-1,-1),'TOP'),
                ('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),4)]))
            output.extend([table,Spacer(1,8)]);continue
        if line.startswith('# '):flush();output.append(Paragraph(inline(line[2:]),styles['Title']))
        elif line.startswith('## '):flush();output.append(Paragraph(inline(line[3:]),styles['Heading2']))
        elif line.startswith('- '):
            flush();paragraph.append('• '+line[2:])
        elif not line.strip():flush()
        else:paragraph.append(line.strip())
        i+=1
    flush()
    return output


def footer(canvas, doc):
    canvas.setFont('DejaVu',7)
    canvas.drawString(40,24,'Tô Văn Chúc • 23020727 • TurtleBot 4 / Nav2 / YOLO • Bản nháp')
    canvas.drawRightString(A4[0]-40,24,str(doc.page))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'docs/REPORT.md')
    parser.add_argument('--output', type=Path, default=ROOT/'docs/REPORT_DRAFT.pdf')
    args = parser.parse_args()
    output = args.output
    content=parse(args.source)
    SimpleDocTemplate(str(output),pagesize=A4,leftMargin=40,rightMargin=40,topMargin=35,bottomMargin=40,
                      title='Tô Văn Chúc 23020727 — TurtleBot 4 Nav2 và YOLO (bản nháp)',
                      author='Tô Văn Chúc — 23020727').build(content,onFirstPage=footer,onLaterPages=footer)
    print(output)


if __name__=='__main__':main()
