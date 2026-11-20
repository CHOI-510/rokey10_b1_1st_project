import { createClient, type SupabaseClient } from "@supabase/supabase-js";

let supabaseClient: SupabaseClient | undefined;

function getSupabaseConfig() {
  const url = process.env.NEXT_PUBLIC_SUPABASE_URL;
  const publishableKey = process.env.NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY;

  if (!url || !publishableKey) {
    throw new Error(
      "Supabase is not configured. Set NEXT_PUBLIC_SUPABASE_URL and NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY in .env.local.",
    );
  }

  return { url, publishableKey };
}

export function createSupabaseClient() {
  if (supabaseClient) {
    return supabaseClient;
  }

  const { url, publishableKey } = getSupabaseConfig();
  supabaseClient = createClient(url, publishableKey);
  return supabaseClient;
}
