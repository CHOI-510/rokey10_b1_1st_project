This is a [Next.js](https://nextjs.org) project using the App Router and Supabase.

## Connect Supabase

1. In your Supabase project, open **Project Settings → API** (or **Connect**) and copy the Project URL and publishable key.
2. Copy `.env.example` to `.env.local` and fill in those values:

   ```env
   NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
   NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY=your-supabase-publishable-key
   ```

3. Restart the development server after changing environment variables.

Use `createSupabaseClient()` from `lib/supabase/client.ts` in a Client Component, or `createSupabaseServerClient()` from `lib/supabase/server.ts` in a Server Component or Route Handler. Keep the `service_role`/secret key out of browser code and `.env.local` out of version control. For client access, enable Row Level Security and add policies for the tables and operations the app needs.

The dashboard is not yet mapped to a Supabase table. Once you share the table names and columns (or a schema export), its cards can be connected to the matching data.

## Getting Started

First, run the development server:

```bash
npm run dev
# or
yarn dev
# or
pnpm dev
# or
bun dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

You can start editing the page by modifying `app/page.tsx`. The page auto-updates as you edit the file.

This project uses [`next/font`](https://nextjs.org/docs/app/building-your-application/optimizing/fonts) to automatically optimize and load [Geist](https://vercel.com/font), a new font family for Vercel.

## Learn More

To learn more about Next.js, take a look at the following resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js features and API.
- [Learn Next.js](https://nextjs.org/learn) - an interactive Next.js tutorial.

You can check out [the Next.js GitHub repository](https://github.com/vercel/next.js) - your feedback and contributions are welcome!

## Conect Supabase(PostGreSQL)
