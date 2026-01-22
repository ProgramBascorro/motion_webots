# docs

This is a Next.js application generated with
[Create Fumadocs](https://github.com/fuma-nama/fumadocs).

Run development server:

```bash
npm run dev
# or
pnpm dev
# or
yarn dev
```

Open http://localhost:3000 with your browser to see the result.

## Explore

In the project, you can see:

- `lib/source.ts`: Code for content source adapter, [`loader()`](https://fumadocs.dev/docs/headless/source-api) provides the interface to access your content.
- `lib/layout.shared.tsx`: Shared options for layouts, optional but preferred to keep.

| Route                     | Description                                            |
| ------------------------- | ------------------------------------------------------ |
| `app/(home)`              | The route group for your landing page and other pages. |
| `app/docs`                | The documentation layout and pages.                    |
| `app/api/search/route.ts` | The Route Handler for search.                          |

### Fumadocs MDX

A `source.config.ts` config file has been included, you can customise different options like frontmatter schema.

Read the [Introduction](https://fumadocs.dev/docs/mdx) for further details.

## Learn More

To learn more about Next.js and Fumadocs, take a look at the following
resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js
  features and API.
- [Learn Next.js](https://nextjs.org/learn) - an interactive Next.js tutorial.
- [Fumadocs](https://fumadocs.dev) - learn about Fumadocs

### MISSING

  Your docs/landing site has a solid foundation but is missing ~60% of what professional robotics team websites typically have. Here's a prioritized breakdown:

  🔴 CRITICAL GAPS (Missing entirely)

  1. Blog/News Section - No way to share updates, competition results, achievements
  2. Media Gallery - Only 1 3D model; no real robot photos, videos, or build process docs
  3. Team Member Profiles - Currently just text; no photos, bios, or individual visibility
  4. Contact Form - Just an email link; no actual form for inquiries/sponsorships
  5. Analytics - No tracking of visitor behavior, no Google Analytics/Search Console
  6. SEO Optimization - Limited meta tags, no sitemap.xml, no structured data (Schema.org)

  🟡 HIGH PRIORITY (Incomplete or minimal)

  1. Event Calendar - Competition dates scattered in docs; needs dedicated calendar widget
  2. Downloads/Resources - Only external links; missing CAD files, setup scripts, tutorials
  3. FAQ Expansion - Only 3 items; should have 20-30 covering tech, recruitment, sponsorship
  4. Team Visibility - Member cards exist but are text-only; need photos and real profiles
  5. Sponsorship Page - Mentioned in footer but doesn't exist; missing sponsor showcase

  🟢 NICE TO HAVE (Enhancement)

  1. Social Integration - GitHub stats, Instagram feed embeds, YouTube integration
  2. Testimonials - Alumni and member success stories
  3. Project Portfolio - Robot evolution, design challenges, lessons learned
  4. Partner Showcase - Hardware vendor/sponsor logos
  5. PWA Features - App-like experience, offline support

  ---
  Quick Implementation Priority (What to Add First)

  Week 1 - Quick Wins:
  - Contact form (use Formspree - 2hrs)
  - Expand FAQ to 20+ items (4hrs)
  - Add Google Analytics (1hr)
  - Create sitemap + robots.txt (1hr)
  - Team member cards with placeholder photos (6hrs)

  Week 2-3:
  - Blog section (/blog/ route, 3-5 launch articles)
  - Media gallery (/gallery/ with robot photos)
  - Event calendar widget
  - Sponsorship tier page
  - Downloads page

  Month 2+:
  - Team database with member profiles
  - Video library (competition matches)
  - Project timeline/evolution
  - Testimonials system

  ---
  Specific Missing Assets/Libraries

  Images:
  - Team member photos (currently none)
  - Robot construction photos
  - Competition photos
  - Team event photos
  - Hardware/component close-ups

  Videos:
  - Competition match highlights
  - Team introduction video
  - Robot demo videos
  - Build process time-lapses

  Libraries to Add:
  # Image gallery
  pnpm add next-image-gallery  # or use built-in <Image>

  # Contact forms
  pnpm add react-hook-form zod  # or use Formspree

  # Calendar
  pnpm add react-calendar  # or use full-calendar

  # CMS (optional)
  pnpm add sanity  # or Strapi for blog/team management

  ---
```
  File Structure for New Content

  docs/content/
  ├── docs/         (✓ exists)
  ├── blog/         (❌ missing) → 2025/competition-debrief.mdx
  └── downloads/    (❌ missing) → CAD files, scripts

  src/app/
  ├── blog/         (❌ missing)
  ├── gallery/      (❌ missing)
  ├── team/         (❌ missing → upgrade from text)
  ├── events/       (❌ missing)
  ├── contact/      (❌ missing)
  ├── downloads/    (❌ missing)
  └── sponsorship/  (❌ missing)
```
  ---
  TL;DR: You have great documentation + landing page. What you need: blog → gallery → team profiles → contact form → analytics → event calendar. That's what separates a "nice tech site" from a "professional organization site."

    SOURCES REVIEWED

  - https://techunited.nl/?page_id=2135&lang=en - RoboCup winners
  - https://robots.htwk-leipzig.de/startseite - 2025 Humanoid champions
  - https://www.team254.com/ - FRC legends
  - https://spl.robocup.org/teams/
  - https://humanoid.robocup.org/
  - https://challenges.robotevents.com/challenge/118/robotics-team-website-challenge
  - https://www.firstinspires.org/resources/library/frc/technical-resources
  - https://www.thebluealliance.com/ - FRC tracking/aggregator

https://github.com/utra-robosoccer/soccerbot