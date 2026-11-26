#!/usr/bin/env python3
"""
design_notes.py - a one-page write-up of the Robo Chess arm's chain + ball-screw joints, for sharing (Mike 2026-11-20:
"do me up a PDF and then convert it into a PNG ... a write-up about this, my design, some of the changes, some of the
things we found, some of the revisions that made things better ... just some change log, basic, quick, not too
verbose"). Writes chain_drive/robo_chess_arm_notes.pdf and .png (Facebook takes the PNG).

    python design_notes.py
"""
import os

import pymupdf as fitz                          # PyMuPDF: the PDF page -> PNG
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import inch
from reportlab.platypus import ListFlowable, ListItem, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "chain_drive")
PDF = os.path.join(OUT, "robo_chess_arm_notes.pdf")
PNG = os.path.join(OUT, "robo_chess_arm_notes.png")

INK, ACCENT, MUTED = colors.HexColor("#1d2330"), colors.HexColor("#2f6db5"), colors.HexColor("#5a6270")
T = ParagraphStyle("t", fontName="Helvetica-Bold", fontSize=19, leading=23, textColor=INK)
SUB = ParagraphStyle("s", fontName="Helvetica", fontSize=10, leading=13, textColor=MUTED, spaceAfter=6)
H = ParagraphStyle("h", fontName="Helvetica-Bold", fontSize=12, leading=15, textColor=ACCENT, spaceBefore=8, spaceAfter=3)
B = ParagraphStyle("b", fontName="Helvetica", fontSize=9.6, leading=12.4, textColor=INK, alignment=TA_LEFT)
SMALL = ParagraphStyle("sm", parent=B, fontSize=8.2, leading=10.4, textColor=MUTED)


def bullets(items):
    return ListFlowable([ListItem(Paragraph(t, B), leftIndent=10, value="circle") for t in items],
                        bulletType="bullet", start="•", leftIndent=12, bulletFontSize=8, bulletColor=ACCENT)


def build():
    story = [
        Paragraph("Robo Chess arm: chain + ball-screw joints", T),
        Paragraph("Mike Overbay's design notes and change log, November 2026", SUB),

        Paragraph("How each joint is driven", H),
        Paragraph("A NEMA 17 sits at the joint end of each link and belts (GT2) to an SFU1204 ball screw lying along "
                  "the link. The ball nut rides in an aluminium carrier that pulls two #25 roller chains. The chains "
                  "wrap a 25-tooth sprocket fixed to the previous link, so the screw's straight pull turns the joint. "
                  "The chain works like a flexible rack wrapped round a pinion.", B),
        Spacer(1, 4),
        Table([["Reduction", "about 40:1 (elbow, J3: belts 1:1), 75:1 at the shoulder (60:32 belt)"],
               ["Backlash", "near zero: ball screw + two tensioned chains"],
               ["Joint travel", "shoulder 2.5–147°, elbow and J3 −104° to +4.6° (the nut's 64 / 48 mm)"],
               ["Link weights", "1.65 / 1.50 / 1.41 kg (links A, B, C, from the model)"],
               ["Lift at full reach", "about 100 g with a 1.5× safety margin; about 1.3 kg before it stalls (slow)"]],
              colWidths=[1.35 * inch, 5.55 * inch],
              style=TableStyle([("FONT", (0, 0), (0, -1), "Helvetica-Bold", 9.2), ("FONT", (1, 0), (1, -1), "Helvetica", 9.2),
                                ("TEXTCOLOR", (0, 0), (-1, -1), INK), ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#eef2f8")),
                                ("LINEBELOW", (0, 0), (-1, -1), 0.4, colors.HexColor("#d5dbe5")),
                                ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3)])),

        Paragraph("Changes that made it better", H),
        bullets([
            "<b>Motors moved to the joint end</b> of each link (they were out at the far end): less weight on a long "
            "lever, more lift.",
            "<b>Chain drives on every joint.</b> The 3D-printed 37:1 cycloid gearbox is shelved until one is built that "
            "survives its load test.",
            "<b>Nut anti-rotation:</b> two small 3×6×2 bearings on top of each nut carrier ride 0.5 mm under "
            "the top plate. The screw's turning force no longer twists the chains (about 1 mm of slop at the claw "
            "without them, about 0.25 mm with).",
            "<b>Free-end plate thickened toward the back</b> and the far idlers moved back one chain link, so their "
            "shafts sit fully in the plastic. The screw's end bearing sits in a pocket on a 1.5 mm shoulder.",
            "<b>One-piece printed frame</b>, printed on its side: floor and top plate flush with the outside; cheeks "
            "reshaped into a 4 mm Z so every part lies flat.",
            "<b>Covers held by M3×4×14 shoulder screws</b> into heat-set inserts in half-capsule bosses on "
            "the outside of the frame. The shoulder locates the cover; 74 for the whole arm.",
            "<b>Idler bosses</b> bear on the bearings' inner race only, so the outer race spins free.",
        ]),

        Paragraph("Things we found", H),
        bullets([
            "The chains can't hold the nut from turning: they pull along the screw, and the nut's torque needs a push "
            "sideways. Hence the guide bearings.",
            "At slow speed the shoulder is limited by the chain (about 25 N·m), not the motor. Every gram on the "
            "arm counts against the shoulder at full reach.",
            "The far idler shafts broke out of their plate by 1 mm. Caught in the model, fixed before printing.",
            "Joint travel has to match the nut's travel, or the animation runs the nut off the screw.",
        ]),

        Paragraph("Robot Studio (the design viewer)", H),
        Paragraph("The chains, screws, nuts and pulleys animate as the joints move. A sequencer builds the arm up "
                  "stage by stage, then it draws on paper with a pen in the claw, with a camera that chases the claw. "
                  "Shadows, coloured lights and photo textures; one side cover left off so you can see inside.", B),
        Spacer(1, 8),
        Paragraph("Parts: NEMA 17 steppers, SFU1204 ball screws (BK/BF-style machined ends), #25 roller chain, 25T "
                  "plate sprockets cut on a home CNC, GT2 belts and pulleys, 625 / 608 / 6801 / 6808 bearings, "
                  "printed frame and covers.", SMALL),
    ]
    os.makedirs(OUT, exist_ok=True)
    doc = SimpleDocTemplate(PDF, pagesize=letter, leftMargin=0.8 * inch, rightMargin=0.8 * inch,
                            topMargin=0.7 * inch, bottomMargin=0.6 * inch, title="Robo Chess arm: chain + ball-screw joints",
                            author="Mike Overbay")
    doc.build(story)
    with fitz.open(PDF) as d:
        if d.page_count != 1:
            print(f"WARNING: {d.page_count} pages")
        d[0].get_pixmap(dpi=200).save(PNG)
    print("wrote", PDF, "and", PNG)


if __name__ == "__main__":
    build()
