import re
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, HRFlowable, Preformatted, Table, TableStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

MD_PATH = "/data/data/com.termux/files/home/13.17/CLAUDE.md"
PDF_PATH = "/data/data/com.termux/files/home/13.17/CLAUDE.pdf"

F = "/usr/share/fonts/truetype/liberation"
pdfmetrics.registerFont(TTFont("Sans",        f"{F}/LiberationSans-Regular.ttf"))
pdfmetrics.registerFont(TTFont("Sans-Bold",   f"{F}/LiberationSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("Sans-Italic", f"{F}/LiberationSans-Italic.ttf"))
pdfmetrics.registerFont(TTFont("Mono",        f"{F}/LiberationMono-Regular.ttf"))
pdfmetrics.registerFont(TTFont("Mono-Bold",   f"{F}/LiberationMono-Bold.ttf"))

ACCENT   = colors.HexColor("#2563EB")   # синий для заголовков
H1_LINE  = colors.HexColor("#2563EB")
DIVIDER  = colors.HexColor("#CBD5E1")
CODE_BG  = colors.HexColor("#F1F5F9")
QUOTE_BG = colors.HexColor("#F8FAFC")
QUOTE_LINE = colors.HexColor("#94A3B8")
TEXT     = colors.HexColor("#1E293B")
MUTED    = colors.HexColor("#64748B")

h1 = ParagraphStyle("h1", fontName="Sans-Bold",   fontSize=18, textColor=TEXT,
                    spaceBefore=4, spaceAfter=2, leading=22)
h2 = ParagraphStyle("h2", fontName="Sans-Bold",   fontSize=13, textColor=TEXT,
                    spaceBefore=14, spaceAfter=3, leading=17)
h3 = ParagraphStyle("h3", fontName="Sans-Bold",   fontSize=11, textColor=MUTED,
                    spaceBefore=8,  spaceAfter=2, leading=14)
normal = ParagraphStyle("normal", fontName="Sans", fontSize=10, textColor=TEXT,
                        spaceAfter=3, leading=15)
bullet = ParagraphStyle("bullet", fontName="Sans", fontSize=10, textColor=TEXT,
                        leftIndent=14, spaceAfter=2, leading=14)
sub_bullet = ParagraphStyle("sub_bullet", fontName="Sans", fontSize=9.5, textColor=MUTED,
                             leftIndent=28, spaceAfter=2, leading=13)
code_style = ParagraphStyle("code", fontName="Mono", fontSize=8.5, textColor=TEXT,
                             backColor=CODE_BG, spaceAfter=6, leading=12,
                             leftIndent=8, rightIndent=8, borderPadding=6)
quote_style = ParagraphStyle("quote", fontName="Sans-Italic", fontSize=9.5, textColor=MUTED,
                              backColor=QUOTE_BG, leftIndent=14, spaceAfter=4, leading=13)

def escape(text):
    text = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    return text

def inline_format(text):
    text = escape(text)
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
    text = re.sub(r"\*(.+?)\*",     r"<i>\1</i>", text)
    text = re.sub(r"`(.+?)`", r'<font name="Mono" size="9" color="#0F172A">\1</font>', text)
    return text

doc = SimpleDocTemplate(
    PDF_PATH, pagesize=A4,
    leftMargin=22*mm, rightMargin=22*mm,
    topMargin=20*mm, bottomMargin=20*mm,
    title="Проект 13.17 — Концепция"
)

story = []
in_code_block = False
code_lines = []

def flush_code():
    if code_lines:
        story.append(Preformatted("\n".join(code_lines), code_style))
        code_lines.clear()

with open(MD_PATH, "r", encoding="utf-8") as f:
    lines = f.readlines()

for line in lines:
    line = line.rstrip("\n")

    if line.startswith("```"):
        if in_code_block:
            flush_code()
            in_code_block = False
        else:
            in_code_block = True
        continue

    if in_code_block:
        code_lines.append(line)
        continue

    if line.startswith("# "):
        story.append(Spacer(1, 2*mm))
        story.append(Paragraph(inline_format(line[2:]), h1))
        story.append(HRFlowable(width="100%", thickness=2, color=H1_LINE, spaceAfter=4))

    elif line.startswith("## "):
        story.append(Paragraph(inline_format(line[3:]), h2))
        story.append(HRFlowable(width="100%", thickness=0.7, color=DIVIDER, spaceAfter=3))

    elif line.startswith("### "):
        story.append(Paragraph(inline_format(line[4:]), h3))

    elif line.startswith("---"):
        story.append(Spacer(1, 3*mm))
        story.append(HRFlowable(width="100%", thickness=0.5, color=DIVIDER, spaceAfter=3*mm))

    elif line.startswith("> "):
        story.append(Paragraph(inline_format(line[2:]), quote_style))

    elif re.match(r"^  [-*] ", line):
        story.append(Paragraph("◦ " + inline_format(line[4:]), sub_bullet))

    elif re.match(r"^[-*] ", line):
        story.append(Paragraph("• " + inline_format(line[2:]), bullet))

    elif line.strip() == "":
        story.append(Spacer(1, 3*mm))

    else:
        story.append(Paragraph(inline_format(line), normal))

flush_code()

doc.build(story)
print(f"PDF создан: {PDF_PATH}")
